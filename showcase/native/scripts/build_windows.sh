#!/bin/bash
# Cross-compile the native showcase for Windows x64 from Linux (mingw, static).
# Needs: rustup target x86_64-pc-windows-gnu, g++-mingw-w64-x86-64-posix, glslang-tools.
# TARMAC_SRC defaults to ../tmcore (the private core is NOT in the public repo; point it at your local copy).
set -euo pipefail
cd "$(dirname "$0")/.."
export PATH="$HOME/.cargo/bin:$PATH"
cargo build --release --target x86_64-pc-windows-gnu "$@"
OUT=../native-win64; mkdir -p "$OUT"
cp target/x86_64-pc-windows-gnu/release/drivechain-native.exe "$OUT/"
echo "-> $OUT/drivechain-native.exe"
