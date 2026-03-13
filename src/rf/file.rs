use std::{
    fs::File,
    io::{BufReader, Seek, SeekFrom},
    path::{Path, PathBuf},
    sync::Arc,
    time::Instant,
};

use byteorder::{LittleEndian, ReadBytesExt};
use num_complex::Complex32;

use crate::{
    normalise::{norm_f32, norm_i16, norm_i8},
    IqBlock, IqSource, RfConfig, RfError, RfResult, SampleFormat, SourceMetrics,
};

/// Источник IQ-данных, читаемый из бинарного файла.
pub struct FileSource {
    path: PathBuf,
    reader: BufReader<File>,
    config: Arc<RfConfig>,
    next_samples: u64,
    looping: bool,
    start_time: Option<Instant>,
    metrics: SourceMetrics,
}

impl FileSource {
    /// Открыть файл с заданной конфигурацией RF.
    pub fn open<P: AsRef<Path>>(
        path: P,
        config: Arc<RfConfig>,
    ) -> RfResult<Self> {
        config.validate()?;

        let path = path.as_ref().to_owned();
        let file = File::open(&path)?;

        Ok(Self {
            path,
            reader: BufReader::with_capacity(1 << 20, file), // 1MiB read buffer
            config,
            next_samples: 0,
            looping: false,
            start_time: None,
            metrics: SourceMetrics::default(),
        })
    }

    /// Включить режим зацикливания: при достижении конца файла чтение
    /// продолжается с начала. Удобно для многократного воспроизведения
    /// коротких сигналов при разработке алгоритмов.
    pub fn with_looping(mut self) -> Self {
        self.looping = true;
        self
    }

    /// Общее количество комплексных сэмплов в файле.
    pub fn total_samples(&self) -> RfResult<u64> {
        let file = File::open(&self.path)?;
        let bytes = file.metadata()?.len();
        let bps = self.config.format.bytes_per_sample() as u64;

        if bytes & bps != 0 {
            log::warn!(
                "file size {} is not a multiple of {} bytes per sample",
                bytes,
                bps,
            );
        }

        Ok(bytes / bps)
    }

    /// Длительность файла в секундах.
    pub fn duration_s(&self) -> RfResult<f64> {
        Ok(self.total_samples()? as f64 / self.config.sample_rate_hz)
    }

    /// Внутреннее чтение блока комплексных сэмплов.
    fn read_block_inner(
        &mut self,
        n: usize,
    ) -> RfResult<Vec<Complex32>> {
        let mut samples = Vec::with_capacity(n);

        match self.config.format {
            SampleFormat::I8 => {
                for _ in 0..n {
                    let i = match self.reader.read_i8() {
                        Ok(v) => v,
                        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
                            return if samples.is_empty() {
                                Err(RfError::EndOfFile)
                            } else {
                                Ok(samples)
                            }
                        }
                        Err(e) => return Err(e.into()),
                    };

                    let q = self.reader.read_i8()?;

                    samples.push(norm_i8(i, q));
                }
            }
            SampleFormat::I16 => {
                for _ in 0..n {
                    let i = match self.reader.read_i16::<LittleEndian>() {
                        Ok(v) => v,
                        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
                            return if samples.is_empty() {
                                Err(RfError::EndOfFile)
                            } else {
                                Ok(samples)
                            }
                        }
                        Err(e) => return Err(e.into()),
                    };

                    let q = self.reader.read_i16::<LittleEndian>()?;

                    samples.push(norm_i16(i, q));
                }
            }
            SampleFormat::F32 => {
                for _ in 0..n {
                    let i = match self.reader.read_f32::<LittleEndian>() {
                        Ok(v) => v,
                        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
                            return if samples.is_empty() {
                                Err(RfError::EndOfFile)
                            } else {
                                Ok(samples)
                            }
                        }
                        Err(e) => return Err(e.into()),
                    };

                    let q = self.reader.read_f32::<LittleEndian>()?;

                    samples.push(norm_f32(i, q));
                }
            }
        }

        Ok(samples)
    }

    /// Перемотка файл в начало.
    fn rewind(&mut self) -> RfResult<()> {
        self.reader.seek(SeekFrom::Start(0))?;

        Ok(())
    }

    /// Обновление метрики скорости доставки сэмплов.
    fn update_rate_metric(
        &mut self,
        delivered: usize,
    ) {
        let now = Instant::now();
        let start = *self.start_time.get_or_insert(now);
        let elapsed = (now - start).as_secs_f64();

        if elapsed > 0.5 {
            // Обновление не чаще чем раз в 0.5 секунды
            let rate = self.metrics.total_samples as f64 / elapsed;

            self.metrics.measured_rate_hz = Some(rate);
        }

        self.metrics.total_samples += delivered as u64;
    }
}

impl IqSource for FileSource {
    fn config(&self) -> &RfConfig {
        &self.config
    }

    fn name(&self) -> &str {
        self.path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("<file>")
    }

    fn read_block(
        &mut self,
        n: usize,
    ) -> RfResult<super::IqBlock> {
        let start_sample = self.next_samples;
        let mut samples: Vec<Complex32> = Vec::with_capacity(n);

        while samples.len() < n {
            let remaining = n - samples.len();

            match self.read_block_inner(remaining) {
                Ok(mut chunk) => samples.append(&mut chunk),
                Err(RfError::EndOfFile) if self.looping && samples.len() < n => {
                    if samples.is_empty() {
                        return Err(RfError::EndOfFile);
                    }

                    log::debug!("FileSource: EOF mid-block, looping");
                    self.rewind()?;
                }
                Err(RfError::EndOfFile) if self.looping => break,
                Err(RfError::EndOfFile) => {
                    if samples.is_empty() {
                        return Err(RfError::EndOfFile);
                    }

                    break;
                }
                Err(e) => return Err(e),
            }
        }

        let len = samples.len() as u64;

        self.next_samples += len;
        self.update_rate_metric(len as usize);

        Ok(IqBlock {
            samples,
            config: Arc::clone(&self.config),
            start_sample,
        })
    }

    fn seek(
        &mut self,
        sample_offset: u64,
    ) -> RfResult<()> {
        let byte_offset = sample_offset * self.config.format.bytes_per_sample() as u64;

        self.reader.seek(SeekFrom::Start(byte_offset))?;
        self.next_samples = sample_offset;

        Ok(())
    }

    fn metrics(&self) -> SourceMetrics {
        self.metrics.clone()
    }
}

#[cfg(test)]
mod tests {
    use std::io::Write;

    use tempfile::NamedTempFile;

    use super::*;

    fn make_i8_file(pairs: &[(i8, i8)]) -> NamedTempFile {
        let mut f = NamedTempFile::new().unwrap();

        for &(i, q) in pairs {
            f.write_all(&i.to_le_bytes()).unwrap();
            f.write_all(&q.to_le_bytes()).unwrap();
        }

        f.flush().unwrap();

        f
    }

    fn make_f32_file(pairs: &[(f32, f32)]) -> NamedTempFile {
        let mut f = NamedTempFile::new().unwrap();
        for &(i, q) in pairs {
            f.write_all(&i.to_le_bytes()).unwrap();
            f.write_all(&q.to_le_bytes()).unwrap();
        }

        f.flush().unwrap();

        f
    }

    fn default_config(format: SampleFormat) -> Arc<RfConfig> {
        Arc::new(RfConfig {
            format,
            ..RfConfig::default()
        })
    }

    #[test]
    fn test_i9_read_correct_values() {
        let f = make_i8_file(&[(127, -127), (0, 0), (-64, 64)]);
        let mut src = FileSource::open(f.path(), default_config(SampleFormat::I8)).unwrap();
        let block = src.read_block(3).unwrap();

        assert_eq!(block.samples.len(), 3);
        assert!((block.samples[0].re - 1.0).abs() < 0.01);
        assert!((block.samples[0].im + 1.0).abs() < 0.01);
        assert_eq!(block.samples[1], Complex32::new(0.0, 0.0));
    }

    #[test]
    fn test_i8_partial_block_at_eof() {
        let f = make_i8_file(&[(1, 2), (3, 4)]);
        let mut src = FileSource::open(f.path(), default_config(SampleFormat::I8)).unwrap();
        let block = src.read_block(10).unwrap();

        assert_eq!(block.samples.len(), 2);
    }

    #[test]
    fn test_i8_eof_error_on_second_call() {
        let f = make_i8_file(&[(1, 2)]);
        let mut src = FileSource::open(f.path(), default_config(SampleFormat::I8)).unwrap();

        src.read_block(1).unwrap();

        assert!(matches!(src.read_block(1), Err(RfError::EndOfFile)));
    }

    #[test]
    fn test_f32_reads_correct_values() {
        let f = make_f32_file(&[(0.5, -0.5), (1.0, 0.0)]);
        let mut src = FileSource::open(f.path(), default_config(SampleFormat::F32)).unwrap();
        let block = src.read_block(2).unwrap();

        assert!((block.samples[0].re - 0.5).abs() < 1e-6);
        assert!((block.samples[0].im + 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_seek_reads_from_offset() {
        let pairs: Vec<(i8, i8)> = (0i8..8).map(|i| (i, i)).collect();
        let f = make_i8_file(&pairs);
        let mut src = FileSource::open(f.path(), default_config(SampleFormat::I8)).unwrap();

        src.seek(4).unwrap();

        let block = src.read_block(1).unwrap();

        assert!((block.samples[0].re - 4.0 / 127.0).abs() < 0.01);
        assert_eq!(block.start_sample, 4);
    }

    #[test]
    fn test_looping_wraps_around() {
        let f = make_i8_file(&[(10, 20), (30, 40)]);
        let mut src = FileSource::open(f.path(), default_config(SampleFormat::I8))
            .unwrap()
            .with_looping();
        let block = src.read_block(3).unwrap();

        assert_eq!(block.samples.len(), 3);

        let first = norm_i8(10, 20);

        assert!((block.samples[2].re - first.re).abs() < 1e-6);
    }

    #[test]
    fn test_total_samples_correct() {
        let f = make_i8_file(&[(0, 0); 1024]);
        let src = FileSource::open(f.path(), default_config(SampleFormat::I8)).unwrap();

        assert_eq!(src.total_samples().unwrap(), 1024);
    }

    #[test]
    fn test_duration_1ms_at_2048k() {
        let f = make_i8_file(&[(0, 0); 2048]);
        let src = FileSource::open(f.path(), default_config(SampleFormat::I8)).unwrap();
        let d = src.duration_s().unwrap();

        assert!((d - 0.001).abs() < 1e-9);
    }

    #[test]
    fn test_metrics_count_delivered_samples() {
        let f = make_i8_file(&[(0, 0); 128]);
        let mut src = FileSource::open(f.path(), default_config(SampleFormat::I8)).unwrap();

        src.read_block(64).unwrap();
        src.read_block(64).unwrap();

        assert_eq!(src.metrics().total_samples, 128);
    }

    #[test]
    fn start_sample_increments_across_blocks() {
        let f = make_i8_file(&[(0, 0); 64]);
        let mut src = FileSource::open(f.path(), default_config(SampleFormat::I8)).unwrap();
        let b1 = src.read_block(32).unwrap();
        let b2 = src.read_block(32).unwrap();

        assert_eq!(b1.start_sample, 0);
        assert_eq!(b2.start_sample, 32);
    }
}
