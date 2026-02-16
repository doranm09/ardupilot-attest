#!/usr/bin/env bash
set -euo pipefail

SWITCH_NAME="${1:-rocq-9.0.1}"
eval "$(opam env --switch="${SWITCH_NAME}")"

printf 'Using OPAM switch: %s\n' "$(opam switch show)"
printf 'coqc: %s\n' "$(command -v coqc)"
coqc --version | head -n 2
printf 'cvm: %s\n' "$(command -v cvm)"
