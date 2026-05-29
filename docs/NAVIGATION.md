# 7. Navigation Layer

Module:

```text
src/navigation/
```

## Название

Navigation-слой декодирует **навигационные сообщения спутников** из
демодулированного потока битов, поступающего из tracking-слоя. На
выходе — структурированные данные об орбите и часах спутника,
необходимые для вычисления псевдодальностей.

## Входные и выходные данные

```text
Tracking (prompt I-компонент, 1 бит / 20 мс)
    │
    ▼
[Navigation Layer]
    │
    ├── Ephemeris  { a, e, i, Ω, ω, M₀, … }
    ├── ClockParams { af0, af1, af2, toc }
    ├── IonosphericModel { α[], β[] }
    └── Almanac    { orbital approx for all PRN }
    │
    ▼
Observables (pseudorange calculator)
```

## Структура навигационного сообщения GPS L1 C/A

### Формат кадра

```
1 кадр = 5 subframes × 300 бит = 1500 бит = 30 секунд

Subframe 1 (300 бит):
  ┌─ TLM (30 бит) ─ HOW (30 бит) ─ слова 3-10 (240 бит) ─┐
  │  преамбула      TOW, subframe ID   clock parameters     │
  └──────────────────────────────────────────────────────────┘

Subframe 2: orbital params (часть 1)
Subframe 3: orbital params (часть 2)
Subframe 4: almanac + ионосфера (страницы 1-25, меняются по кругу)
Subframe 5: almanac (PRN 1-24)
```

### TLM и HOW

| Поле            | Размер | Описание                                                  |
| --------------- | ------ | --------------------------------------------------------- |
| TLM преамбула   | 8 бит  | `0b10001011` = `0x8B` — признак начала subframe           |
| TLM message     | 14 бит | зарезервировано                                           |
| HOW TOW         | 17 бит | время начала следующего subframe (в 6-секундных единицах) |
| HOW subframe ID | 3 бит  | 1–5 — номер текущего subframe                             |

### Контроль чётности

Каждое 30-битное слово содержит 6-битный контроль чётности (Hamming):

```text
биты 1-24: данные
биты 25-30: parity
```

Декодирование: проверить каждое из 6 уравнений чётности. При ошибке — ждать
следующего subframe.

## Компоненты

### FrameDecoder (`frame_decoder.rs`)

**Задачи:**

1. Детекция преамбулы TLM (`0x8B`)
2. Проверка parity каждого слова
3. Извлечение TOW и номера subframe из HOW
4. Сборка полного subframe (10 × 30 бит)
5. Диспетчеризация в парсеры Subframe 1/2/3

```rust
pub struct FrameDecoder {
    bit_buffer: VecDeque<u8>,
    state: DecodeState,
}

pub enum DecodeState {
    SearchingPreamble,
    VerifyingAlignment { offset: usize },
    Collecting { subframe_id: u8, bits_collected: usize },
}
```

**Алгоритм детекции преамбулы:**

```text
for offset in 0..30:
    if bit_buffer[offset..offset+8] == 0b10001011:
        if parity(word_at(offset)) == OK:
            locked = true
```

### EphemerisParser (`ephemeris.rs`)

Декодирует Subframe 1, 2, 3 в соответствии с GPS ICD-200.

#### Subframe 1 — параметры часов

| Параметр      | Биты   | Масштаб   | Описание                   |
| ------------- | ------ | --------- | -------------------------- |
| `week_number` | 10 бит | 1         | Номер GPS недели           |
| `ura_index`   | 4 бит  | —         | User Range Accuracy        |
| `sv_health`   | 6 бит  | —         | 0 = исправен               |
| `iodc`        | 10 бит | 1         | Issue of Data Clock        |
| `toc`         | 16 бит | 2⁴ с      | Время для параметров часов |
| `af2`         | 8 бит  | 2⁻⁵⁵ с/с² | Квадратичный коэф. часов   |
| `af1`         | 16 бит | 2⁻⁴³ с/с  | Линейный коэф. часов       |
| `af0`         | 22 бит | 2⁻³¹ с    | Постоянный коэф. часов     |

#### Subframe 2 — орбита (часть 1)

| Параметр  | Биты   | Масштаб    | Описание                   |
| --------- | ------ | ---------- | -------------------------- |
| `iode`    | 8 бит  | 1          | Issue of Data Ephemeris    |
| `crs`     | 16 бит | 2⁻⁵ м      | Поправка синуса радиуса    |
| `delta_n` | 16 бит | 2⁻⁴³ рад/с | Поправка среднего движения |
| `m0`      | 32 бит | 2⁻³¹ п рад | Средняя аномалия           |
| `cuc`     | 16 бит | 2⁻²⁹ рад   | Поправка широты (cos)      |
| `e`       | 32 бит | 2⁻³³       | Эксцентриситет             |
| `cus`     | 16 бит | 2⁻²⁹ рад   | Поправка широты (sin)      |
| `sqrt_a`  | 32 бит | 2⁻¹⁹ м½    | Корень большой полуоси     |
| `toe`     | 16 бит | 2⁴ с       | Время эфемерид             |

#### Subframe 3 — орбита (часть 2)

| Параметр    | Биты   | Масштаб    | Описание                  |
| ----------- | ------ | ---------- | ------------------------- |
| `cic`       | 16 бит | 2⁻²⁹ рад   | Поправка наклонения (cos) |
| `omega0`    | 32 бит | 2⁻³¹ п рад | Долгота восходящего узла  |
| `cis`       | 16 бит | 2⁻²⁹ рад   | Поправка наклонения (sin) |
| `i0`        | 32 бит | 2⁻³¹ п рад | Наклонение                |
| `crc`       | 16 бит | 2⁻⁵ м      | Поправка радиуса (cos)    |
| `omega`     | 32 бит | 2⁻³¹ п рад | Аргумент перигея          |
| `omega_dot` | 24 бит | 2⁻⁴³ рад/с | Скорость изменения узла   |
| `idot`      | 14 бит | 2⁻⁴³ рад/с | Скорость изменения i₀     |

### Вычисление позиции спутника (ECEF)

По данным эфемерид, для момента времени `t`:

```text
1. Вычислить среднее движение:
   n₀ = √(μ / a³),   μ = 3.986005×10¹⁴ м³/с²
   n  = n₀ + Δn

2. Среднее время от TOE:
   tk = t − toe  (с учётом переполнения недели)

3. Средняя аномалия:
   Mk = M₀ + n·tk

4. Эксцентрическая аномалия Ek (итерации Кеплера):
   Ek = Mk + e·sin(Ek)  (≈5 итераций)

5. Истинная аномалия:
   νk = atan2(√(1−e²)·sin(Ek), cos(Ek)−e)

6. Аргумент широты:
   Φk = νk + ω

7. Поправки:
   δuk = cus·sin(2Φk) + cuc·cos(2Φk)
   δrk = crs·sin(2Φk) + crc·cos(2Φk)
   δik = cis·sin(2Φk) + cic·cos(2Φk)

8. Исправленные значения:
   uk = Φk + δuk
   rk = a·(1 − e·cos(Ek)) + δrk
   ik = i₀ + δik + idot·tk

9. Позиция в плоскости орбиты:
   xk' = rk·cos(uk)
   yk' = rk·sin(uk)

10. Долгота восходящего узла:
    Ωk = Ω₀ + (Ω̇ − Ω̇e)·tk − Ω̇e·toe
    Ω̇e = 7.2921151467×10⁻⁵ рад/с

11. ECEF координаты:
    x = xk'·cos(Ωk) − yk'·cos(ik)·sin(Ωk)
    y = xk'·sin(Ωk) + yk'·cos(ik)·cos(Ωk)
    z = yk'·sin(ik)
```

### Ionospheric Model (`nav_data.rs`)

Модель Клобухара из Subframe 4 (страница 18):

```text
Параметры: α₀ α₁ α₂ α₃ (амплитуда)
           β₀ β₁ β₂ β₃ (период)

Коррекция (секунды):
  T_iono = F × (5×10⁻⁹ + A·cos(2π(t−50400)/P))
  F = 1 + 16(0.53−El)³

где El — угол возвышения спутника в полукругах,
    A = Σαₙ·φₙ  (до нуля, если < 0),
    P = Σβₙ·φₙ  (не менее 72000 с)
```

### NavData (`nav_data.rs`)

Хранилище текущего состояния навигационных данных:

```rust
pub struct NavData {
    /// Эфемериды по каждому PRN (1-32 для GPS)
    pub ephemeris: HashMap<u8, Ephemeris>,
    /// Параметры ионосферной модели
    pub iono: Option<IonosphericModel>,
    /// Альманах (приближённые орбиты всех спутников)
    pub almanac: HashMap<u8, AlmanacEntry>,
    /// GPS–UTC поправка (leapseconds)
    pub utc_correction: Option<UtcCorrection>,
}
```

**Валидация эфемерид:**

- Проверка `sv_health == 0` (бит здоровья)
- Проверка `IODE == IODC` (согласованность данных)
- Проверка возраста данных: `|t − toe| < 2 часа`

## Поток декодирования (1 эпоха = 20 мс)

```text
tracking: I_prompt (бит навигационного сообщения)
    │
    ▼  знак I → бит (> 0 → 1, < 0 → 0)
bit_buffer.push(bit)
    │
    ▼  каждые 30 бит
check_parity(word)
    │ OK
    ▼  каждые 10 слов (300 бит)
decode_subframe(bits[0..300])
    │
    ├── subframe_id == 1 → parse_clock_params()    → NavData.clock
    ├── subframe_id == 2 → parse_orbit_part1()     ┐
    ├── subframe_id == 3 → parse_orbit_part2()     ┘→ NavData.ephemeris[prn]
    └── subframe_id == 4 → parse_iono_or_almanac() → NavData.iono / .almanac
```

**Время до первого fix:**

- Subframe 1–3 декодируются за 18 с (3 × 6 с)
- Полный набор эфемерид: ~30 с
- С корректным альманахом warm start: ~6 с

## Файловая структура

```text
src/navigation/
├── mod.rs              — экспорты, NavigationState
├── frame_decoder.rs    — бит-синхронизация, parity, сборка subframe
├── ephemeris.rs        — декодирование SF1/SF2/SF3, позиция спутника
└── nav_data.rs         — NavData, IonosphericModel, Almanac, UtcCorrection
```

## Связь с другими модулями

| От кого  | Что получает          | Что отдаёт       | Кому                          |
| -------- | --------------------- | ---------------- | ----------------------------- |
| Tracking | I_prompt (бит, 20 мс) | —                | —                             |
| —        | —                     | Ephemeris        | Observables (pseudorange)     |
| —        | —                     | IonosphericModel | Observables (iono correction) |
| —        | —                     | AlmanacEntry     | Acquisition (быстрый поиск)   |

## Целевые показатели

| Метрика                     | Цель                              |
| --------------------------- | --------------------------------- |
| Bit sync                    | < 40 мс (2 бита)                  |
| Frame sync (preambule lock) | < 6 с                             |
| Subframe 1–3 decode         | < 30 с                            |
| Parity error rate           | < 0.1% при CN0 > 35 дБ-Гц         |
| Позиция спутника, ошибка    | < 1 м при возрасте эфемерид < 2 ч |
