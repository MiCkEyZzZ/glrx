//! File-based IQ source.
//!
//! Reads raw interleaved I/Q samples from files in three formats:
//! [`SampleFormat::I8`], [`SampleFormat::I16`], and [`SampleFormat::F32`].
//!
//! # File format
//!
//! The file is a flat sequence of interleaved `[I, Q, I, Q, ...]` values.
//! No header is present; format, sample rate, and center frequency must be
//! supplied via [`RfConfig`].
//!
//! # Example
//!
//! ```no_run
//! use glrx::rf::{file::FileSource, IqSource, RfConfig, SampleFormat};
//!
//! let config = RfConfig {
//!     center_freq_hz: 1_575_420_000.0,
//!     sample_rate_hz: 2_048_000.0,
//!     format: SampleFormat::I8,
//!     ..Default::default()
//! };
//! let mut src = FileSource::open("gps_l1_2048k.bin", config).unwrap();
//! let block = src.read_block(2048).unwrap(); // 1 ms of GPS L1 C/A
//! ```

use std::{
    fs::File,
    io::{BufReader, Seek, SeekFrom},
    path::{Path, PathBuf},
    time::Instant,
};

use byteorder::{LittleEndian, ReadBytesExt};
use num_complex::Complex32;

use crate::{
    normalise::{norm_f32, norm_i16, norm_i8},
    IqBlock, IqSource, RfConfig, RfError, RfResult, SampleFormat, SourceMetrics,
};

/// IQ source backed by a binary file.
pub struct FileSource {
    path: PathBuf,
    reader: BufReader<File>,
    config: RfConfig,
    /// Monotonically-increasing sample counter.
    next_samples: u64,
    /// Whether to loop the file on EOF instead of returning
    /// [`RfError::EndOfFile`].
    looping: bool,
    /// Time of the first call to `read_block`, for rate measurement.
    start_time: Option<Instant>,
    metrics: SourceMetrics,
}

impl FileSource {
    /// Open a file with the given RF configuration.
    pub fn open<P: AsRef<Path>>(
        path: P,
        config: RfConfig,
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

    /// Enable looping: when EOF is reached the file is rewound and reading
    /// continues from the beginning. Useful for repeating short signal captures
    /// during algorithm development.
    #[must_use]
    pub const fn with_looping(mut self) -> Self {
        self.looping = true;
        self
    }

    /// Total number of complex samples in the file.
    pub fn total_samples(&self) -> RfResult<u64> {
        let file = File::open(&self.path)?;
        let bytes = file.metadata()?.len();
        let bps = self.config.format.bytes_per_complex_sample() as u64;

        if bytes % bps != 0 {
            log::warn!(
                "file size {} is not a multiple of {} bytes per sample",
                bytes,
                bps,
            );
        }

        Ok(bytes / bps)
    }

    /// Duration of the file in seconds.
    pub fn duration_s(&self) -> RfResult<f64> {
        Ok(self.total_samples()? as f64 / self.config.sample_rate_hz)
    }

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

    fn rewind(&mut self) -> RfResult<()> {
        self.reader.seek(SeekFrom::Start(0))?;

        Ok(())
    }

    fn update_rate_metric(
        &mut self,
        delivered: usize,
    ) {
        let now = Instant::now();
        let start = *self.start_time.get_or_insert(now);
        let elapsed = (now - start).as_secs_f64();

        if elapsed > 0.5 {
            // Update at most every 0.5 s
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
                        // Nothing read at all before EOF — file is truly empty
                        return Err(RfError::EndOfFile);
                    }

                    log::debug!("FileSource: EOF mid-block, looping");
                    self.rewind()?;
                    // continue loop to fill the rest
                }
                Err(RfError::EndOfFile) if self.looping => break,
                Err(RfError::EndOfFile) => {
                    if samples.is_empty() {
                        return Err(RfError::EndOfFile);
                    }

                    break; // partial block at end of file
                }
                Err(e) => return Err(e),
            }
        }

        let len = samples.len() as u64;

        self.next_samples += len;
        self.update_rate_metric(len as usize);

        Ok(IqBlock {
            samples,
            config: self.config.clone(),
            start_sample,
        })
    }

    fn seek(
        &mut self,
        sample_offset: u64,
    ) -> RfResult<()> {
        let byte_offset = sample_offset * self.config.format.bytes_per_complex_sample() as u64;

        self.reader.seek(SeekFrom::Start(byte_offset))?;
        self.next_samples = sample_offset;

        Ok(())
    }

    fn metrics(&self) -> SourceMetrics {
        self.metrics.clone()
    }
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use std::io::Write;

    use tempfile::NamedTempFile;

    use super::*;

    fn make_i8_file(pairs: &[(i8, i8)]) -> NamedTempFile {
        let mut f = NamedTempFile::new().unwrap();
        for &(i, q) in pairs {
            f.write_all(&[i as u8, q as u8]).unwrap();
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

    fn default_config(format: SampleFormat) -> RfConfig {
        RfConfig {
            format,
            ..RfConfig::default()
        }
    }

    #[test]
    fn i8_reads_correct_values() {
        let f = make_i8_file(&[(127, -127), (0, 0), (-64, 64)]);
        let mut src = FileSource::open(f.path(), default_config(SampleFormat::I8)).unwrap();
        let block = src.read_block(3).unwrap();
        assert_eq!(block.samples.len(), 3);
        assert!((block.samples[0].re - 1.0).abs() < 0.01);
        assert!((block.samples[0].im + 1.0).abs() < 0.01);
        assert_eq!(block.samples[1], Complex32::new(0.0, 0.0));
    }

    #[test]
    fn i8_partial_block_at_eof() {
        // Write only 2 samples but ask for 10.
        let f = make_i8_file(&[(1, 2), (3, 4)]);
        let mut src = FileSource::open(f.path(), default_config(SampleFormat::I8)).unwrap();
        let block = src.read_block(10).unwrap();
        assert_eq!(block.samples.len(), 2);
    }

    #[test]
    fn i8_eof_error_on_second_call() {
        let f = make_i8_file(&[(1, 2)]);
        let mut src = FileSource::open(f.path(), default_config(SampleFormat::I8)).unwrap();
        src.read_block(1).unwrap();
        assert!(matches!(src.read_block(1), Err(RfError::EndOfFile)));
    }

    #[test]
    fn f32_reads_correct_values() {
        let f = make_f32_file(&[(0.5, -0.5), (1.0, 0.0)]);
        let mut src = FileSource::open(f.path(), default_config(SampleFormat::F32)).unwrap();
        let block = src.read_block(2).unwrap();
        assert!((block.samples[0].re - 0.5).abs() < 1e-6);
        assert!((block.samples[0].im + 0.5).abs() < 1e-6);
    }

    #[test]
    fn seek_reads_from_offset() {
        let pairs: Vec<(i8, i8)> = (0i8..8).map(|i| (i, i)).collect();
        let f = make_i8_file(&pairs);
        let mut src = FileSource::open(f.path(), default_config(SampleFormat::I8)).unwrap();
        // Skip first 4 samples and read 1
        src.seek(4).unwrap();
        let block = src.read_block(1).unwrap();
        assert!((block.samples[0].re - 4.0 / 127.0).abs() < 0.01);
        assert_eq!(block.start_sample, 4);
    }

    #[test]
    fn looping_wraps_around() {
        let f = make_i8_file(&[(10, 20), (30, 40)]);
        let mut src = FileSource::open(f.path(), default_config(SampleFormat::I8))
            .unwrap()
            .with_looping();
        // Read 3 samples: 2 from file + 1 looped
        let block = src.read_block(3).unwrap();
        assert_eq!(block.samples.len(), 3);
        // Third sample should equal the first
        let first = norm_i8(10, 20);
        assert!((block.samples[2].re - first.re).abs() < 1e-6);
    }

    #[test]
    fn total_samples_correct() {
        let f = make_i8_file(&[(0, 0); 1024]);
        let src = FileSource::open(f.path(), default_config(SampleFormat::I8)).unwrap();
        assert_eq!(src.total_samples().unwrap(), 1024);
    }

    #[test]
    fn duration_1ms_at_2048k() {
        let f = make_i8_file(&[(0, 0); 2048]);
        let src = FileSource::open(f.path(), default_config(SampleFormat::I8)).unwrap();
        let d = src.duration_s().unwrap();
        assert!((d - 0.001).abs() < 1e-9);
    }

    #[test]
    fn metrics_count_delivered_samples() {
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
