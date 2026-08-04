set shell := ["bash", "-euo", "pipefail", "-c"]

# Crates exercised by `test-cpu` (anything that doesn't need a GPU).
cpu_crates := "-p monstertruck-core -p monstertruck-traits -p monstertruck-geometry -p monstertruck-topology -p monstertruck-mesh -p monstertruck-meshing -p monstertruck-modeling -p monstertruck-solid -p monstertruck-healing -p monstertruck-fillet -p monstertruck-step"

# Crates exercised by `test-gpu`.
gpu_crates := "-p monstertruck-gpu -p monstertruck-render"

# RUSTFLAGS required for the wasm32 target.
wasm_rustflags := '--cfg=web_sys_unstable_apis --cfg=getrandom_backend="wasm_js"'

# Default: show available recipes.
default:
    @just --list

# Aggregate: what CI runs.
ci: fmt-check lint-check readme-check test-cpu test-doc meshing-features

# Format code.
fmt:
    cargo fmt --all

# Verify formatting without writing.
fmt-check:
    cargo fmt --all -- --check

# Run clippy with autofix (modifies working tree).
lint:
    cargo clippy --fix --allow-dirty --allow-staged --all-targets -- -D warnings

# Run clippy without fixing (CI-safe).
lint-check:
    cargo clippy --all-targets -- -D warnings

# Run CPU-only tests on the stable toolchain.
#
# `cargo nextest run`, not `cargo test`: nextest gives each test its own PROCESS.
# Some tests here read process-global measurement counters (the
# `parameter_division` work meter in `monstertruck-traits`), and under `cargo
# test`'s threads-in-one-process model a concurrently running test charges the
# same counter, so the assertion can never hold. Nextest does NOT run doctests --
# `test-doc` covers those separately, and `ci` runs both.
test-cpu:
    cargo nextest run {{ cpu_crates }} --features derive --features polynomial

# Doctests. Nextest cannot run these, so they are their own step.
test-doc:
    cargo test --doc {{ cpu_crates }} --features derive --features polynomial

# Run CPU-only tests on the nightly toolchain.
test-cpu-nightly:
    rustup run nightly cargo nextest run {{ cpu_crates }} --features derive --features polynomial

# Run GPU tests (requires a working GPU). Serialized: these create real wgpu
# devices, which do not tolerate concurrent construction.
test-gpu:
    cargo nextest run {{ gpu_crates }} -j1 --no-capture

# Feature subset build checks for `monstertruck-meshing`.
meshing-features:
    cargo check -p monstertruck-meshing --no-default-features --features analyzers
    cargo check -p monstertruck-meshing --no-default-features --features filters
    cargo check -p monstertruck-meshing --no-default-features --features tessellation

# Build the workspace for the `wasm32-unknown-unknown` target.
wasm-build:
    RUSTFLAGS='{{ wasm_rustflags }}' cargo build --target=wasm32-unknown-unknown

# Build the workspace for wasm32 with the `webgl` feature.
webgl-build:
    RUSTFLAGS='{{ wasm_rustflags }}' cargo build --target=wasm32-unknown-unknown --features webgl

# Build and run the JS/Deno tests for `monstertruck-wasm`.
wasm-js-test:
    RUSTFLAGS='--cfg=getrandom_backend="wasm_js"' bash -c '\
        cd monstertruck-wasm && \
        wasm-pack build --target web && \
        deno test -A tests/'

# Full wasm test suite: wasm32 build + webgl build + JS tests.
wasm-test: wasm-build webgl-build wasm-js-test

# Build the ad-hoc viewer (wasm-pack + bootstrap files).
adhoc-viewer:
    RUSTFLAGS='--cfg=getrandom_backend="wasm_js"' bash -c '\
        cd monstertruck-wasm && \
        wasm-pack build --target web && \
        cp examples/index.html pkg/ && \
        cp examples/bootstrap.js pkg/ && \
        cp examples/script.js pkg/'

# Build the WebGPU example pages.
wgpu-examples:
    RUSTFLAGS='{{ wasm_rustflags }}' cargo run --bin example-pages-generator

# Build everything that ships in the GitHub Pages site.
page-build: adhoc-viewer wgpu-examples

# Generate shape JSON fixtures used by examples and tests.
create-shape-json:
    cd resources/shape && \
    cargo run -p monstertruck-modeling --example bottle && \
    cargo run -p monstertruck-modeling --example cube && \
    cargo run -p monstertruck-modeling --example cylinder && \
    cargo run -p monstertruck-modeling --example punched-cube && \
    cargo run -p monstertruck-modeling --example torus-punched-cube && \
    cargo run -p monstertruck-modeling --example cube-in-cube && \
    cargo run -p monstertruck-modeling --example torus && \
    cargo run -p monstertruck-modeling --example sphere && \
    cargo run -p monstertruck-modeling --example torus -- 500 100 large-torus.json && \
    cargo run -p monstertruck-solid --example punched-cube-shapeops

# Build rustdoc for every crate (no deps).
doc:
    cargo doc --no-deps --workspace

# cargo-rdme's output is NOT stable across major versions: v1.5 left stripped
# intralinks as broken markdown (`` `Shell`(monstertruck_topology::Shell) ``)
# where v2 removes them properly. CI pins the exact version below, so pin the
# same one locally or `readme-check` will disagree with CI.
rdme_version := "2.1.0"

# Crates whose README is generated from their crate-level docs by `cargo rdme`.
# `monstertruck-derive` is deliberately absent: it does the opposite, pulling its
# README into its docs via `#![doc = include_str!("../README.md")]`, so pointing
# rdme at it would make the two definitions circular.
rdme_crates := "monstertruck monstertruck-assembly monstertruck-core monstertruck-fillet monstertruck-geometry monstertruck-gpu monstertruck-healing monstertruck-mesh monstertruck-meshing monstertruck-modeling monstertruck-render monstertruck-solid monstertruck-step monstertruck-topology monstertruck-traits monstertruck-wasm"

# Regenerate every generated README from its crate-level docs.
#
# The crate docs are the single source of truth; everything outside the
# `<!-- cargo-rdme start -->`/`<!-- cargo-rdme end -->` markers (attribution,
# license) is preserved. `--heading-base-level 1` shifts the docs' `# Examples`
# to `##` so it sits under the README's title.
readme:
    #!/usr/bin/env bash
    set -euo pipefail
    have="$(cargo rdme --version | awk '{ print $2 }')"
    if [ "$have" != "{{ rdme_version }}" ]; then
        printf 'cargo-rdme {{ rdme_version }} required, found %s.\n' "$have" >&2
        printf 'Install it with: cargo install cargo-rdme --version {{ rdme_version }}\n' >&2
        exit 1
    fi
    for crate in {{ rdme_crates }}; do
        cargo rdme -w "$crate" --heading-base-level 1 --intralinks-strip-links --force
    done

# Verify every generated README matches its crate docs. Exit 3 on mismatch.
#
# `--intralinks-strip-links` renders intra-doc links as plain text instead of
# resolving them to docs.rs URLs. That is deliberate: resolving them makes
# cargo-rdme shell out to a PINNED nightly (`nightly-2026-06-22`), which turns a
# README check into a toolchain dependency and fails on stable CI. The generated
# output is byte-identical either way -- verified -- so nothing is lost, and the
# intra-doc links keep working on docs.rs where they actually resolve.
readme-check:
    #!/usr/bin/env bash
    set -uo pipefail
    have="$(cargo rdme --version | awk '{ print $2 }')"
    if [ "$have" != "{{ rdme_version }}" ]; then
        printf 'cargo-rdme {{ rdme_version }} required, found %s.\n' "$have" >&2
        printf 'Install it with: cargo install cargo-rdme --version {{ rdme_version }}\n' >&2
        exit 1
    fi
    status=0
    for crate in {{ rdme_crates }}; do
        if ! cargo rdme -w "$crate" --heading-base-level 1 --intralinks-strip-links --check; then
            printf 'README out of date: %s (run `just readme`)\n' "$crate" >&2
            status=1
        fi
    done
    exit "$status"

# Wipe target directory.
clean:
    cargo clean
