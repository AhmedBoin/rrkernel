#!/usr/bin/env bash
#
# Runs the portable core *and* the POSIX backend of rrkernel on Linux (or WSL):
#
#   bash scripts/run-linux.sh
#
# `CARGO_TARGET_DIR` defaults to /tmp so a shared Windows build directory is
# never polluted with Linux artifacts (and vice versa).
set -euo pipefail

cd "$(dirname "$0")/.."
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-/tmp/rrkernel-linux-target}"

# Make the script work from a non-login shell too (which is what WSL/CI invoke).
if ! command -v cargo >/dev/null 2>&1 && [ -f "$HOME/.cargo/env" ]; then
    # shellcheck disable=SC1091
    . "$HOME/.cargo/env"
fi
if ! command -v cargo >/dev/null 2>&1; then
    echo "cargo not found. Install it with:" >&2
    echo "  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y" >&2
    exit 1
fi

echo "=== unit / invariant tests (host target, POSIX backend compiled in) ==="
cargo test --features std

echo
echo "=== backend smoke test: switch, automatic exit, timer ==="
timeout 60 cargo run --quiet --example smoke --features std --release

echo
echo "=== round-robin demo: preemption, automatic unlink, dynamic spawn ==="
# A timeout so a regression in the scheduler shows up as a failing CI step
# instead of a hung job.
timeout 120 cargo run --quiet --example roundrobin_demo --features std --release

echo
echo "=== slice-accuracy benchmark ==="
timeout 180 cargo run --quiet --example jitter_bench --features std --release

echo
echo "all Linux backend checks completed"
