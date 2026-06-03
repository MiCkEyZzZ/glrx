# Навигационный слой

Модуль:

```text
src/navigation/
```

## Обзор

Навигационный слой декодирует **сообщения спутниковой навигации** из
демодулированного битового потока, создаваемого слоем слежения. Выходные данные
представляют собой структурированные данные об орбите спутника и времени,
необходимые для вычисления псевдодальности.

## Входные и выходные данные

```text
Tracking (prompt I-component, 1 бит / 20 ms)
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
Observables (калькулятор псевдодальности)
```

## Структура навигационного сообщения GPS L1 C/A

### Формат кадра

```text
1 frame = 5 subframes × 300 bits = 1500 bits = 30 seconds

Subframe 1 (300 бит):
  ┌─ TLM (30 bits) ─ HOW (30 bits) ─ words 3-10 (240 bits) ─┐
  │  preamble      TOW, subframe ID   clock parameters      │
  └─────────────────────────────────────────────────────────┘

Subframe 2: orbital params (part 1)
Subframe 3: orbital params (part 2)
Subframe 4: almanac + ionosphere (pages 1-25, cyclic)
Subframe 5: almanac (PRN 1-24)
```

### TLM и HOW

| Поле            | Размер | Описание                                        |
| --------------- | ------ | ----------------------------------------------- |
| TLM preamble    | 8 бит  | `0b10001011` = `0x8B` — subframe start marker   |
| TLM message     | 14 бит | reserved                                        |
| HOW TOW         | 17 бит | start time of next subframe (in 6-second units) |
| HOW subframe ID | 3 бит  | 1–5 — current subframe number                   |

### Проверка четности

Каждое 30-битное слово содержит 6-битное поле четности (Хэмминга):

```text
bits 1-24: data
bits 25-30: parity
```

Декодирование: проверяет каждое из 6 уравнений четности. В случае ошибки дождается
следующего подкадра.

## Компоненты

### FrameDecoder (`frame_decoder.rs`)

**Обязанности:**

1. Обнаружение преамбулы TLM (`0x8B`)
2. Проверка четности каждого слова
3. Извлечение идентификатора TOW и подкадра из HOW
4. Сборка полного подкадра (10 × 30 бит)
5. Передача парсерам подкадра 1/2/3

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

**Алгоритм обнаружения преамбулы:**

```text
for offset in 0..30:
    if bit_buffer[offset..offset+8] == 0b10001011:
        if parity(word_at(offset)) == OK:
            locked = true
```

### EphemerisParser (`ephemeris.rs`)

Декодирует подкадры 1, 2, 3 в соответствии с GPS ICD-200.

#### Подкадр 1 — параметры тактирования

| Параметр      | Биты    | Масштаб   | Описание                                |
| ------------- | ------- | --------- | --------------------------------------- |
| `week_number` | 10 бит  | 1         | Номер недели GPS                        |
| `ura_index`   | 4 бита  | —         | Точность диапазона пользователя         |
| `sv_health`   | 6 бит   | —         | 0 = здоровых                            |
| `iodc`        | 10 бит  | 1         | Issue of Data Clock                     |
| `toc`         | 16 бит  | 2⁴ s      | Clock reference time                    |
| `af2`         | 8 бит   | 2⁻⁵⁵ с/с² | Квадратичный коэффициент часов          |
| `af1`         | 16 бит  | 2⁻⁴³ с/с  | Линейный коэффициент часов              |
| `af0`         | 22 бита | 2⁻³¹ с    | Постоянный коэффициент тактовой частоты |

#### Подкадр 2 — Orbit (Part 1)

| Параметр  | Биты    | Масштаб    | Описание                       |
| --------- | ------- | ---------- | ------------------------------ |
| `iode`    | 8 бит   | 1          | Issue of Data Ephemeris        |
| `crs`     | 16 бит  | 2⁻⁵ m      | Radius sine correction         |
| `delta_n` | 16 бит  | 2⁻⁴³ rad/s | Mean motion correction         |
| `m0`      | 32 бита | 2⁻³¹ π rad | Mean anomaly                   |
| `cuc`     | 16 бит  | 2⁻²⁹ rad   | Latitude correction (cos)      |
| `e`       | 32 бита | 2⁻³³       | Eccentricity                   |
| `cus`     | 16 бит  | 2⁻²⁹ rad   | Latitude correction (sin)      |
| `sqrt_a`  | 32 бита | 2⁻¹⁹ m½    | Square root of semi-major axis |
| `toe`     | 16 бит  | 2⁴ s       | Ephemeris reference time       |

#### Подкадр 3 — Orbit (Part 2)

| Параметр    | Биты    | Масштаб    | Описание                     |
| ----------- | ------- | ---------- | ---------------------------- |
| `cic`       | 16 бит  | 2⁻²⁹ rad   | Inclination correction (cos) |
| `omega0`    | 32 бита | 2⁻³¹ π rad | Longitude of ascending node  |
| `cis`       | 16 бит  | 2⁻²⁹ rad   | Inclination correction (sin) |
| `i0`        | 32 бита | 2⁻³¹ π rad | Inclination                  |
| `crc`       | 16 бит  | 2⁻⁵ m      | Radius correction (cos)      |
| `omega`     | 32 бита | 2⁻³¹ π rad | Argument of perigee          |
| `omega_dot` | 24 бита | 2⁻⁴³ rad/s | Rate of right ascension      |
| `idot`      | 14 бит  | 2⁻⁴³ rad/s | Inclination rate             |

### Вычисление положения спутника (ECEF)

Использование эфемеридных данных для времени `t`:

```text
1. Вычисление среднего движения:
   n₀ = √(μ / a³),   μ = 3.986005×10¹⁴ m³/s²
   n  = n₀ + Δn

2. Время с момента TOE:
   tk = t − toe  (с корректировкой переноса недели)

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

8. Скорректированные значения:
   uk = Φk + δuk
   rk = a·(1 − e·cos(Ek)) + δrk
   ik = i₀ + δik + idot·tk

9. Положение в плоскости орбиты:
   xk' = rk·cos(uk)
   yk' = rk·sin(uk)

10. Долгота восходящего узла:
    Ωk = Ω₀ + (Ω̇ − Ω̇e)·tk − Ω̇e·toe
    Ω̇e = 7.2921151467×10⁻⁵ rad/s

11. Координаты ECEF:
    x = xk'·cos(Ωk) − yk'·cos(ik)·sin(Ωk)
    y = xk'·sin(Ωk) + yk'·cos(ik)·cos(Ωk)
    z = yk'·sin(ik)
```

### Ионосферная модель (`nav_data.rs`)

Модель Клобучара из Подкадр 4 (страница 18):

```text
Параметры: α₀ α₁ α₂ α₃ (амплитуда)
            β₀ β₁ β₂ β₃ (период)

Коррекция (секунды):
  T_iono = F × (5×10⁻⁹ + A·cos(2π(t−50400)/P))
  F = 1 + 16(0.53−El)³

где El — высота спутника в полукругах,
      A = Σαₙ·φₙ  (если отрицательное значение, то значение устанавливается равным нулю.),
      P = Σβₙ·φₙ  (минимум 72000 с)
```

### NavData (`nav_data.rs`)

Хранилище для текущего состояния навигационных данных:

```rust
pub struct NavData {
    /// Эфемериды для каждого PRN (1-32 для GPS)
    pub ephemeris: HashMap<u8, Ephemeris>,
    /// Параметры ионосферной модели
    pub iono: Option<IonosphericModel>,
    /// Альманах (приблизительные орбиты для всех спутников)
    pub almanac: HashMap<u8, AlmanacEntry>,
    /// Коррекция GPS–UTC (високосные секунды)
    pub utc_correction: Option<UtcCorrection>,
}
```

**Проверка эфемерид:**

- Check `sv_health == 0` (флаг здоровья)
- Check `IODE == IODC` (согласованность данных)
- Check data age: `|t − toe| < 2 hours`

## Decoding Flow (1 epoch = 20 ms)

```text
tracking: I_prompt (бит навигационного сообщения)
    │
    ▼  I sign → bit (> 0 → 1, < 0 → 0)
bit_buffer.push(bit)
    │
    ▼  every 30 bits
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

**Время для первого ремонта:**

- Декодирование субкадров 1–3 за 18 с (3 × 6 с)
- Полный набор эфемерид: ~30 с
- Теплый старт с действительным альманахом: ~6 с

## Структура файлов

```text
src/navigation/
├── mod.rs              — экспорт, NavigationState
├── frame_decoder.rs    — битовая синхронизация, четность, сборка субкадров
├── ephemeris.rs        — декодирование SF1/SF2/SF3, положение спутника
└── nav_data.rs         — NavData, IonosphericModel, Almanac, UtcCorrection
```

## Интеграция с другими модулями

| От       | Получает              | Производит       | Кому                          |
| -------- | --------------------- | ---------------- | ----------------------------- |
| Tracking | I_prompt (bit, 20 ms) | —                | —                             |
| —        | —                     | Ephemeris        | Observables (pseudorange)     |
| —        | —                     | IonosphericModel | Observables (iono correction) |
| —        | —                     | AlmanacEntry     | Acquisition (fast search)     |

## Целевые показатели

| Метрика                    | Цель                           |
| -------------------------- | ------------------------------ |
| Bit sync                   | < 40 мс (2 бита)               |
| Frame sync (preamble lock) | < 6 с                          |
| Subframe 1–3 decode        | < 30 с                         |
| Parity error rate          | < 0.1% при CN0 > 35 дБ-Гц      |
| Satellite position error   | < 1 м with ephemeris age < 2 ч |
