#!/usr/bin/env bash
# Builds the Bevy visualizer (with the real firmware inside) for the browser
# into web/pkg. Needs: rustup target add wasm32-unknown-unknown, and
# cargo install wasm-bindgen-cli --version 0.2.130 (must match Cargo.lock).
set -euo pipefail
cd "$(dirname "$0")/.."
RUSTFLAGS='--cfg getrandom_backend="wasm_js"' \
  cargo build --profile wasm --target wasm32-unknown-unknown -p ironloop-viz
wasm-bindgen --no-typescript --target web --out-dir web/pkg --out-name ironloop \
  target/wasm32-unknown-unknown/wasm/ironloop-viz.wasm
gzip -9 -k -f web/pkg/ironloop_bg.wasm
ls -lh web/pkg
