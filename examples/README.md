# GLRX Examples

This crate contains standalone execution examples for validating and exploring
core GNSS receiver components in GLRX.

Examples are primarily used for:

- verifying correctness of core algorithms
- observing control loop dynamics (PLL/DLL)
- sanity-checking discriminator signs and loop stability
- debugging signal processing behavior in isolation

These are **not full RF or end-to-end receiver simulations**. Some examples may
be minimal, synthetic, or deterministic by design, while others can represent
more complete integration scenarios.

## Running examples

Run any example binary:

```bash
cargo run -p glrx-examples --bin <example_name>
```

L
