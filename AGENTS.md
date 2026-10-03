# AGENTS.md — Coding agent guidance for triggerbox

The Rust code lives in two cargo workspaces:

- `braid-triggerbox-rs` — the host library (`braid-triggerbox`) and the
  protocol crate shared with the firmware (`braid-triggerbox-comms`).
- `hardware_v3/braid-triggerbox-firmware-pico` — the Raspberry Pi Pico firmware
  (target `thumbv6m-none-eabi`).

## Before declaring a task done

Run every step with the current stable toolchain (`rustup update stable`); CI
uses the latest stable, so new lints land there first. CI runs the same checks;
skipping them will fail CI.

In `braid-triggerbox-rs`:

1. `cargo fmt --all` — format the code. (`cargo fmt --all --check` is the CI
   gate.)
2. `cargo clippy --workspace --all-targets -- -D warnings` — no lints, no
   warnings.
3. `cargo clippy -p braid-triggerbox-comms --no-default-features --features defmt --target thumbv6m-none-eabi -- -D warnings`
   — the protocol crate must also be lint-free without `std`.
4. `cargo test --workspace`
5. `RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps`
6. `cargo deny check` (needs `cargo install cargo-deny`; config in
   `deny.toml` at the repository root).

In `hardware_v3/braid-triggerbox-firmware-pico`:

1. `cargo fmt`
2. `cargo clippy --release -- -D warnings`
3. `cargo build --release`
4. `cargo deny check`

CI also checks that each crate builds with the `rust-version` it declares
(`cargo hack check --rust-version --workspace` in `braid-triggerbox-rs`) and
that every `.rs` file starts with
`// SPDX-License-Identifier: MIT OR Apache-2.0`.

## Defensive programming

The lint configuration (`[workspace.lints]` in `braid-triggerbox-rs/Cargo.toml`
and `[lints]` in the firmware `Cargo.toml`) is deliberately strict:
`unsafe_code` is forbidden, clippy `pedantic` is on, and so are lints against
code which can panic (`unwrap_used`, `expect_used`, `panic`, `indexing_slicing`,
`arithmetic_side_effects`, ...). Release builds keep integer overflow checks.

- Data from the other side of the USB link is untrusted. Never panic on it:
  return an error, or log a warning and drop the data.
- Prefer slice patterns, `get()`, `checked_*`, `saturating_*` and `From`/
  `TryFrom` over indexing, raw arithmetic and `as` casts.
- If a lint must be silenced, use `#[expect(lint, reason = "...")]` on the
  smallest possible item, never `#[allow]`. Lints which are noise in tests are
  relaxed in `clippy.toml`.
- Do not add `rustfmt.toml` overrides.

## Version control

- Every commit made by a model should have the model name in the commit summary
  and full message.
- Check if the checkout is setup to use `jj` by checking the output of
  `jj status`. If so, prefer `jj` commands over `git`.
- Use [conventional commits](https://www.conventionalcommits.org/) (`feat:`,
  `fix:`, `docs:`, `refactor:`, `chore(deps):`, `ci:`, `build:`, ...), with `!`
  for breaking changes. release-plz uses them to choose version bumps and
  git-cliff uses them to write the changelogs.
- Keep `Cargo.lock` changes in their own commits.
