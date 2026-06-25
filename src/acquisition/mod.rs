//! Слой захвата сигнала — поиск спутников GPS L1 C/A.
//!
//! # Алгоритм PCPS (Parallel Code Search)
//!
//! Для каждой пробной частоты Доплера `f_d` из сетки поиска:
//!
//! ```text
//! wiped[n]  = signal[n] × exp(−j·2π·f_d·n/fs)   — снятие несущей
//! S[k]      = FFT(wiped)
//! C[k]      = FFT(prn_код)                        — предвычислено
//! power[n]  = |IFFT(S[k] × conj(C[k]))|²         — корреляционная поверхность
//! peak      = argmax(power)  →  code_phase        — фаза кода
//! ```
//!
//! # Структура модуля
//!
//! | Файл | Ответственность |
//! |------|-----------------|
//! | `fft_search`  | Ядро PCPS, двумерная поверхность (Доплер × фаза кода) |
//! | `detector`    | CFAR-обнаружение, оценка C/N₀, валидация второго пика |
//! | `verifier`    | Двухпроходная верификация, политика повтора, Rayon-параллелизм |
//! | `correlator`  | (дублирует fft_search — см. вопрос об объединении)   |

pub mod correlator;
pub mod detector;
pub mod fft_search;
pub mod verifier;
