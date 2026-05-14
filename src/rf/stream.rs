//! Потоковый кольцевой буфер для IQ-данных.
//!
//! Модуль предоставляет:
//! - `IqStream` — фабрику для создания пары производитель/потребитель
//! - `StreamProducer` — запись IQ-блоков в буфер
//! - `StreamConsumer` — чтение IQ-блоков через [`IqSource`]
//! - `OverflowPolicy` — политику обработки переполнения буфера

use std::{
    sync::{
        atomic::{AtomicU64, AtomicUsize, Ordering},
        Arc,
    },
    time::{Duration, Instant},
};

use num_complex::Complex32;
use parking_lot::Mutex;

use crate::{IqBlock, IqSource, RfConfig, RfError, RfResult, SourceMetrics};

/// Разделяемая конфигурация RF-тракта.
pub type SharedRfConfig = Arc<RfConfig>;

/// Политика поведения, когда кольцевой буфер заполнен и производитель пытается
/// записать новые данные.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverflowPolicy {
    /// Бесшумно удалить самый старый слот; производитель не блокируется.
    DropOldest,

    /// Вернуть ошибку [`RfError::BufferOverflow`] производителю.
    ErrorOnOverflow,

    /// Заблокировать производителя до освобождения слота.
    ///
    /// Не подходит для систем жёсткого реального времени.
    BlockProducer,
}

struct Slot {
    /// IQ-выборки, хранящиеся в этом слоте.
    samples: Vec<Complex32>,

    /// Индекс первого сэмпла в этом слоте.
    start_sample: u64,

    /// Момент записи данных; используется для обнаружения разрывов потока.
    written_at: Instant,
}

/// Внутренняя структура кольцевого буфера.
///
/// Разделяется между производителем и потребителем через [`Arc`].
/// Хранит слоты с IQ-данными, атомарные индексы головы и хвоста, а также
/// статистику потока.
///
/// Не предназначена для прямого использования; доступ осуществляется через
/// [`StreamProducer`] и [`StreamConsumer`].
struct SharedBuffer {
    slots: Vec<Mutex<Option<Slot>>>,
    capacity: usize,
    slot_size: usize,
    head: AtomicUsize,
    tail: AtomicUsize,
    count: AtomicUsize,
    total_written: AtomicU64,
    total_read: AtomicU64,
    dropped: AtomicU64,
    interruptions: AtomicU64,
    policy: OverflowPolicy,
    config: SharedRfConfig,
    gap_threshold: Duration,
}

/// Сторона производителя кольцевого буфера.
///
/// Обычно принадлежит потоку захвата SDR или задаче предварительной загрузки из
/// файла.
pub struct StreamProducer {
    buf: Arc<SharedBuffer>,
}

/// Сторона потребителя кольцевого буфера.
///
/// Реализует [`IqSource`], поэтому может быть подключена к любому этапу
/// конвейера обработки.
pub struct StreamConsumer {
    buf: Arc<SharedBuffer>,
    last_read_at: Option<Instant>,
}

/// Фабрика для создания потокового кольцевого буфера IQ.
///
/// [`IqStream`] создаёт пару:
/// - [`StreamProducer`] — записывает IQ-данные
/// - [`StreamConsumer`] — читает IQ-данные и реализует [`IqSource`]
///
/// Обычно используется как граница между:
///
/// ```text
/// SDR / File reader  →  StreamProducer
/// DSP pipeline       ←  StreamConsumer
/// ```
pub struct IqStream;

impl SharedBuffer {
    fn new(
        config: SharedRfConfig,
        capacity: usize,
        slot_size: usize,
        policy: OverflowPolicy,
    ) -> Self {
        let slots = (0..capacity).map(|_| Mutex::new(None)).collect();

        Self {
            slots,
            capacity,
            slot_size,
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            count: AtomicUsize::new(0),
            total_written: AtomicU64::new(0),
            total_read: AtomicU64::new(0),
            dropped: AtomicU64::new(0),
            interruptions: AtomicU64::new(0),
            policy,
            config,
            gap_threshold: Duration::from_millis(5),
        }
    }

    fn used_slots(&self) -> usize {
        self.count.load(Ordering::Acquire)
    }

    fn is_full(&self) -> bool {
        self.count.load(Ordering::Acquire) >= self.capacity
    }

    fn is_empty(&self) -> bool {
        self.count.load(Ordering::Acquire) == 0
    }
}

impl StreamProducer {
    /// Записывает блок IQ-выборок в кольцевой буфер.
    ///
    /// `samples` копируются во внутренний слот буфера.
    ///
    /// Если буфер заполнен, поведение зависит от [`OverflowPolicy`]:
    /// - [`OverflowPolicy::DropOldest`] — самый старый слот удаляется
    /// - [`OverflowPolicy::ErrorOnOverflow`] — возвращается ошибка
    /// - [`OverflowPolicy::BlockProducer`] — производитель ждёт освобождения
    ///   слота
    ///
    /// # Параметры
    ///
    /// * `samples` — IQ-выборки
    /// * `start_sample` — индекс первого сэмпла
    ///
    /// # Ошибки
    ///
    /// Возвращает [`RfError::BufferOverflow`], если используется политика
    /// [`OverflowPolicy::ErrorOnOverflow`].
    pub fn write(
        &self,
        samples: &[Complex32],
        start_sample: u64,
    ) -> RfResult<()> {
        // Handle overflow
        if self.buf.is_full() {
            match self.buf.policy {
                OverflowPolicy::DropOldest => {
                    let dropped = self.buf.slot_size as u64;

                    self.buf.dropped.fetch_add(dropped, Ordering::Relaxed);

                    // Evict the oldest slot: clear it and advance tail
                    let t = self.buf.tail.load(Ordering::Acquire);

                    {
                        let mut slot = self.buf.slots[t].lock();

                        *slot = None;
                    }

                    self.buf
                        .tail
                        .store((t + 1) % self.buf.capacity, Ordering::Release);
                    self.buf.count.fetch_sub(1, Ordering::AcqRel);
                    log::debug!("IqStream: buffer full, dropped {dropped} samples");
                }
                OverflowPolicy::ErrorOnOverflow => {
                    let dropped = samples.len();

                    self.buf
                        .dropped
                        .fetch_add(dropped as u64, Ordering::Relaxed);

                    return Err(RfError::BufferOverflow { dropped });
                }
                OverflowPolicy::BlockProducer => {
                    let deadline = Instant::now() + Duration::from_secs(5);

                    while self.buf.is_full() {
                        if Instant::now() > deadline {
                            return Err(RfError::BufferOverflow {
                                dropped: samples.len(),
                            });
                        }

                        std::hint::spin_loop();
                    }
                }
            }
        }

        let h = self.buf.head.load(Ordering::Acquire);

        {
            let mut slot = self.buf.slots[h].lock();

            *slot = Some(Slot {
                samples: samples.to_vec(),
                start_sample,
                written_at: Instant::now(),
            });
        }

        self.buf
            .head
            .store((h + 1) % self.buf.capacity, Ordering::Release);
        self.buf.count.fetch_add(1, Ordering::AcqRel);
        self.buf
            .total_written
            .fetch_add(samples.len() as u64, Ordering::Relaxed);

        Ok(())
    }

    /// Возвращает приблизительное количество занятых слотов.
    #[must_use]
    pub fn used_slots(&self) -> usize {
        self.buf.used_slots()
    }

    /// Возвращает количество сэмплов, отброшенных из-за переполнения.
    #[must_use]
    pub fn dropped_samples(&self) -> u64 {
        self.buf.dropped.load(Ordering::Relaxed)
    }
}

impl StreamConsumer {
    /// Возвращает количество слотов, доступных для чтения.
    #[must_use]
    pub fn available_slots(&self) -> usize {
        self.buf.used_slots()
    }

    /// Извлекает один слот из кольцевого буфера.
    ///
    /// Функция блокируется до тех пор, пока:
    /// - в буфере не появятся данные, либо
    /// - не истечёт `timeout`.
    fn drain_one_slot(
        &mut self,
        timeout: Duration,
    ) -> RfResult<Slot> {
        let deadline = Instant::now() + timeout;

        loop {
            if !self.buf.is_empty() {
                break;
            }

            if Instant::now() > deadline {
                return Err(RfError::Io(std::io::Error::new(
                    std::io::ErrorKind::TimedOut,
                    "IqStream: timeout waiting for data",
                )));
            }

            std::hint::spin_loop();
        }

        let t = self.buf.tail.load(Ordering::Acquire);
        let slot = self.buf.slots[t]
            .lock()
            .take()
            .expect("slot should contain data");

        self.buf
            .tail
            .store((t + 1) % self.buf.capacity, Ordering::Release);
        self.buf.count.fetch_sub(1, Ordering::AcqRel);
        self.buf
            .total_read
            .fetch_add(slot.samples.len() as u64, Ordering::Relaxed);

        // Обнаружение пробелов
        let now = slot.written_at;

        if let Some(last) = self.last_read_at {
            let expected =
                Duration::from_secs_f64(self.buf.slot_size as f64 / self.buf.config.sample_rate_hz);
            let actual = now.saturating_duration_since(last);
            if actual > expected + self.buf.gap_threshold {
                self.buf.interruptions.fetch_add(1, Ordering::Relaxed);
                log::warn!(
                    "IqStream: stream gap detected (expected {:?}, actual {:?})",
                    expected,
                    actual
                );
            }
        }

        self.last_read_at = Some(now);

        Ok(slot)
    }
}

impl IqStream {
    /// Создаёт новый потоковый кольцевой буфер IQ.
    ///
    /// # Возвращает
    ///
    /// Кортеж:
    /// - [`StreamProducer`] — используется источником данных
    /// - [`StreamConsumer`] — используется DSP-пайплайном
    #[must_use]
    pub fn create(
        config: SharedRfConfig,
        capacity: usize,
        slot_size: usize,
        policy: OverflowPolicy,
    ) -> (StreamProducer, StreamConsumer) {
        assert!(capacity >= 2, "capacity must be at least 2");
        assert!(slot_size > 0, "slot_size must be positive");

        let buf = Arc::new(SharedBuffer::new(config, capacity, slot_size, policy));

        (
            StreamProducer {
                buf: Arc::clone(&buf),
            },
            StreamConsumer {
                buf,
                last_read_at: None,
            },
        )
    }
}

impl IqSource for StreamConsumer {
    fn config(&self) -> &RfConfig {
        self.buf.config.as_ref()
    }

    fn name(&self) -> &str {
        "iq_stream"
    }

    fn read_block(
        &mut self,
        n: usize,
    ) -> RfResult<IqBlock> {
        let timeout = Duration::from_secs(2);
        let mut samples: Vec<Complex32> = Vec::with_capacity(n);
        let mut start_sample = 0u64;

        while samples.len() < n {
            let slot = self.drain_one_slot(timeout)?;

            if samples.is_empty() {
                start_sample = slot.start_sample;
            }

            let needed = n - samples.len();

            if slot.samples.len() <= needed {
                samples.extend_from_slice(&slot.samples);
            } else {
                // Извлекаем из этого слота только то, что нам нужно (частичное истощение)
                samples.extend_from_slice(&slot.samples[..needed]);
                // Примечание: оставшаяся часть теряется — в производственной системе вы можете
                // вернуть её на прежнее место или использовать курсор. Допустимо для
                // выравнивания по блокам.
                log::debug!(
                    "IqStream: partial slot consumed ({} / {} samples used)",
                    needed,
                    slot.samples.len()
                );
                break;
            }
        }

        Ok(IqBlock {
            samples,
            config: Arc::clone(&self.buf.config),
            start_sample,
        })
    }

    fn metrics(&self) -> super::SourceMetrics {
        let total = self.buf.total_read.load(Ordering::Relaxed);
        let dropped = self.buf.dropped.load(Ordering::Relaxed);
        let interruptions = self.buf.interruptions.load(Ordering::Relaxed);
        let measured_rate_hz = if total > 0 {
            Some(self.buf.config.sample_rate_hz)
        } else {
            None
        };

        SourceMetrics {
            total_samples: total,
            dropped_samples: dropped,
            interruptions,
            measured_rate_hz,
            power_dbfs: None,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// Тесты
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    const SLOT: usize = 64;
    const ZERO: Complex32 = Complex32::new(0.0, 0.0);
    const ONE: Complex32 = Complex32::new(1.0, 0.0);

    fn make_stream(
        capacity: usize,
        policy: OverflowPolicy,
    ) -> (StreamProducer, StreamConsumer) {
        IqStream::create(Arc::new(RfConfig::default()), capacity, SLOT, policy)
    }

    #[test]
    fn test_basic_write_read() {
        let (p, mut c) = make_stream(4, OverflowPolicy::DropOldest);
        let data: Vec<Complex32> = vec![ONE; SLOT];

        p.write(&data, 0).unwrap();

        let block = c.read_block(SLOT).unwrap();

        assert_eq!(block.samples.len(), SLOT);
        assert_eq!(block.samples[0], ONE);
        assert_eq!(block.start_sample, 0);
    }

    #[test]
    fn test_multiple_slots_across_reads() {
        let (p, mut c) = make_stream(8, OverflowPolicy::DropOldest);

        for i in 0..3u64 {
            let data = vec![Complex32::new(i as f32, 0.0); SLOT];
            p.write(&data, i * SLOT as u64).unwrap();
        }

        for i in 0..3u64 {
            let block = c.read_block(SLOT).unwrap();

            assert_eq!(block.samples[0].re, i as f32);
            assert_eq!(block.start_sample, i * SLOT as u64);
        }
    }

    #[test]
    fn test_drop_oldest_on_overflow() {
        let (p, mut c) = make_stream(2, OverflowPolicy::DropOldest);

        // Заполнить сверх вместимости
        for i in 0..4u64 {
            p.write(&vec![Complex32::new(i as f32, 0.0); SLOT], i * SLOT as u64)
                .unwrap();
        }

        // Необходимо записать несколько капель
        assert!(p.dropped_samples() > 0, "expected drops");

        // Потребитель всё равно что-то получает
        let block = c.read_block(SLOT).unwrap();

        assert_eq!(block.samples.len(), SLOT);
    }

    #[test]
    fn test_error_on_overflow_policy() {
        let (p, _c) = make_stream(2, OverflowPolicy::ErrorOnOverflow);

        p.write(&vec![ZERO; SLOT], 0).unwrap();
        p.write(&vec![ZERO; SLOT], SLOT as u64).unwrap();

        // Третья запись должна завершиться неудачей
        let result = p.write(&vec![ZERO; SLOT], (2 * SLOT) as u64);

        assert!(matches!(result, Err(RfError::BufferOverflow { .. })));
    }

    #[test]
    fn test_read_larger_than_slot() {
        let (p, mut c) = make_stream(8, OverflowPolicy::DropOldest);

        // Записываем два слота
        p.write(&vec![Complex32::new(1.0, 0.0); SLOT], 0).unwrap();
        p.write(&vec![Complex32::new(2.0, 0.0); SLOT], SLOT as u64)
            .unwrap();

        // Считываем значение, вдвое превышающее размер слота
        let block = c.read_block(SLOT * 2).unwrap();

        assert_eq!(block.samples.len(), SLOT * 2);
    }

    #[test]
    fn test_metrics_track_reads_and_drops() {
        let (p, mut c) = make_stream(2, OverflowPolicy::DropOldest);

        for i in 0..5u64 {
            let _ = p.write(&vec![ZERO; SLOT], i * SLOT as u64);
        }

        let _ = c.read_block(SLOT).unwrap();
        let m = c.metrics();

        assert_eq!(m.total_samples, SLOT as u64);
        assert!(m.dropped_samples > 0);
    }

    #[test]
    fn test_used_slots_increases_on_write() {
        let (p, _c) = make_stream(8, OverflowPolicy::ErrorOnOverflow);

        assert_eq!(p.used_slots(), 0);

        p.write(&vec![ZERO; SLOT], 0).unwrap();

        assert_eq!(p.used_slots(), 1);

        p.write(&vec![ZERO; SLOT], SLOT as u64).unwrap();

        assert_eq!(p.used_slots(), 2);
    }

    #[test]
    fn test_available_slots_decreases_on_read() {
        let (p, mut c) = make_stream(4, OverflowPolicy::DropOldest);

        p.write(&vec![ZERO; SLOT], 0).unwrap();
        p.write(&vec![ZERO; SLOT], SLOT as u64).unwrap();

        assert_eq!(c.available_slots(), 2);

        c.read_block(SLOT).unwrap();

        assert_eq!(c.available_slots(), 1);
    }

    #[test]
    fn test_start_sample_preserved() {
        let (p, mut c) = make_stream(4, OverflowPolicy::DropOldest);
        p.write(&vec![ZERO; SLOT], 12345).unwrap();
        let block = c.read_block(SLOT).unwrap();
        assert_eq!(block.start_sample, 12345);
    }
}
