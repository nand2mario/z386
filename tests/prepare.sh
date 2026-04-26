#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST_DIR="$SCRIPT_DIR/singlestep_real"
REAL_MODE_DIR="$DEST_DIR/v1_ex_real_mode"
REPO_URL="https://github.com/SingleStepTests/80386.git"
PROTECTED_DEST_DIR="$SCRIPT_DIR/singlestep_protected"
PROTECTED_REPO_URL="https://github.com/nand2mario/SingleStepTests_80386_protected.git"

if [[ ! -d "$DEST_DIR/.git" ]]; then
    rm -rf "$DEST_DIR"
    git clone "$REPO_URL" "$DEST_DIR"
fi

if compgen -G "$REAL_MODE_DIR/*.gz" > /dev/null; then
    echo Running gunzip...
    gunzip -f "$REAL_MODE_DIR"/*.gz
fi

if [[ ! -d "$PROTECTED_DEST_DIR/.git" ]]; then
    rm -rf "$PROTECTED_DEST_DIR"
    git clone "$PROTECTED_REPO_URL" "$PROTECTED_DEST_DIR"
fi

echo Test data preparation done.