#!/usr/bin/env bash
# Drop-in replacement for `clang-format` used by the editor's format-on-save.
# Speaks both invocation protocols the cpp/clang-format extensions use:
# plain formatted stdout, and -output-replacements-xml (offset/length edits).
# See tools/insert_function_separators.py for the actual logic.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SCRIPT_DIR/insert_function_separators.py" "$@"
