#!/usr/bin/env python3
"""
Reads already clang-formatted C++ source on stdin and writes it back to
stdout with a separator comment inserted before every top-level (global- or
namespace-scope) function definition:

    <blank line>
    //////////////////////////////////////////////////////////////////////////
    <blank line>
    ReturnType Class::method(...)
    {
        ...
    }

Detection is purely lexical (brace/token scanning that is comment- and
string-literal-aware) — it never touches non-whitespace, non-separator
content. A self-check at the end verifies that invariant and aborts
(passing the input through unchanged) if it would be violated, so a missed
or wrong classification can only result in "no separator added", never in
corrupted code.
"""

import re
import sys

SEPARATOR = "/" * 74
SEPARATOR_RE = re.compile(r"^/{10,}$")  # recognizes any pre-existing separator (any length)

KEYWORD_NAMESPACE = {"namespace"}
KEYWORD_AGGREGATE = {"class", "struct", "union", "enum"}
KEYWORD_CONTROL = {"if", "for", "while", "switch", "catch", "do", "else", "try"}
TRAILING_QUALIFIERS = {"const", "override", "final", "volatile"}
OPERATOR_SYMBOL_CHARS = set("=<>!+-*/%^&|~[]()")

IDENT_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


class Token:
    __slots__ = ("text", "line", "is_comment")

    def __init__(self, text, line, is_comment=False):
        self.text = text
        self.line = line
        self.is_comment = is_comment


def tokenize(text):
    tokens = []
    i = 0
    n = len(text)
    line = 1
    while i < n:
        c = text[i]
        if c == "\n":
            line += 1
            i += 1
            continue
        if c in " \t\r":
            i += 1
            continue
        # line comment
        if c == "/" and i + 1 < n and text[i + 1] == "/":
            j = text.find("\n", i)
            j = n if j == -1 else j
            tokens.append(Token(text[i:j], line, is_comment=True))
            i = j
            continue
        # block comment
        if c == "/" and i + 1 < n and text[i + 1] == "*":
            j = text.find("*/", i + 2)
            j = n if j == -1 else j + 2
            chunk = text[i:j]
            tokens.append(Token(chunk, line, is_comment=True))
            line += chunk.count("\n")
            i = j
            continue
        # raw string literal R"delim(...)delim"
        if c == "R" and i + 1 < n and text[i + 1] == '"':
            m = re.match(r'R"([^()\\ \t]{0,16})\(', text[i:i + 32])
            if m:
                delim = m.group(1)
                end_marker = ")" + delim + '"'
                j = text.find(end_marker, i)
                j = n if j == -1 else j + len(end_marker)
                chunk = text[i:j]
                tokens.append(Token(chunk, line, is_comment=True))
                line += chunk.count("\n")
                i = j
                continue
        # string literal (with escapes)
        if c == '"':
            j = i + 1
            while j < n and text[j] != '"':
                if text[j] == "\\" and j + 1 < n:
                    j += 2
                else:
                    j += 1
            j = min(j + 1, n)
            chunk = text[i:j]
            tokens.append(Token(chunk, line, is_comment=True))
            line += chunk.count("\n")
            i = j
            continue
        # char literal
        if c == "'":
            j = i + 1
            while j < n and text[j] != "'":
                if text[j] == "\\" and j + 1 < n:
                    j += 2
                else:
                    j += 1
            j = min(j + 1, n)
            chunk = text[i:j]
            tokens.append(Token(chunk, line, is_comment=True))
            i = j
            continue
        # identifier / keyword
        if c.isalpha() or c == "_":
            j = i + 1
            while j < n and (text[j].isalnum() or text[j] == "_"):
                j += 1
            tokens.append(Token(text[i:j], line))
            i = j
            continue
        # preprocessor directive: treat whole line as opaque/comment-like
        if c == "#":
            j = text.find("\n", i)
            j = n if j == -1 else j
            tokens.append(Token(text[i:j], line, is_comment=True))
            i = j
            continue
        # single-char punctuation token
        tokens.append(Token(c, line))
        i += 1
    return tokens


def match_paren_backward(stmt, close_idx):
    depth = 0
    k = close_idx
    while k >= 0:
        t = stmt[k].text
        if t == ")":
            depth += 1
        elif t == "(":
            depth -= 1
            if depth == 0:
                return k
        k -= 1
    return None


def looks_like_function_signature(stmt):
    if not stmt:
        return False
    i = len(stmt) - 1
    while i >= 0:
        t = stmt[i].text
        if t in TRAILING_QUALIFIERS:
            i -= 1
            continue
        if t == ")":
            j = match_paren_backward(stmt, i)
            if j is not None and j > 0 and stmt[j - 1].text == "noexcept":
                i = j - 2
                continue
        break
    if i < 0 or stmt[i].text != ")":
        return False
    open_idx = match_paren_backward(stmt, i)
    if open_idx is None or open_idx == 0:
        return False
    before = stmt[open_idx - 1].text
    if before == ">" or (IDENT_RE.match(before) and before not in KEYWORD_CONTROL):
        return True
    k = open_idx - 1
    while k >= 0 and stmt[k].text and all(ch in OPERATOR_SYMBOL_CHARS for ch in stmt[k].text):
        k -= 1
    if k >= 0 and stmt[k].text == "operator":
        return True
    return False


def classify_brace(stmt, enclosing_kind):
    if enclosing_kind in ("FUNCTION", "OTHER"):
        return "OTHER"
    if not stmt:
        return "OTHER"
    first = stmt[0].text
    if first in KEYWORD_NAMESPACE:
        return "NAMESPACE"
    if first in KEYWORD_AGGREGATE:
        return "AGGREGATE"
    if first == "extern":
        return "NAMESPACE"
    if first in KEYWORD_CONTROL:
        return "OTHER"
    if looks_like_function_signature(stmt):
        return "FUNCTION"
    return "OTHER"


def find_function_ranges(tokens):
    """Returns a list of (sig_start_line, close_brace_line) for top-level
    (global- or namespace-scope) function *definitions* only."""
    ranges = []
    frame_stack = [("GLOBAL", None)]
    stmt = []
    for tok in tokens:
        if tok.is_comment:
            continue
        if tok.text == "{":
            enclosing_kind = frame_stack[-1][0]
            kind = classify_brace(stmt, enclosing_kind)
            sig_line = None
            if kind == "FUNCTION" and enclosing_kind in ("GLOBAL", "NAMESPACE"):
                sig_line = stmt[0].line
            frame_stack.append((kind, sig_line))
            stmt = []
            continue
        if tok.text == "}":
            if len(frame_stack) > 1:
                kind, sig_line = frame_stack.pop()
                if kind == "FUNCTION" and sig_line is not None:
                    ranges.append((sig_line, tok.line))
            stmt = []
            continue
        if tok.text == ";":
            stmt = []
            continue
        stmt.append(tok)
    return ranges


def apply_separators(text):
    tokens = tokenize(text)
    ranges = find_function_ranges(tokens)
    if not ranges:
        return text

    sig_start_lines = sorted(line for line, _ in ranges)
    lines = text.split("\n")

    target_lines = set(sig_start_lines)
    out = []
    i = 0
    n = len(lines)
    while i < n:
        line_no = i + 1
        if line_no in target_lines:
            while out and out[-1].strip() == "":
                out.pop()
            while out and SEPARATOR_RE.match(out[-1].strip()):
                out.pop()
                while out and out[-1].strip() == "":
                    out.pop()
            if out:
                out.append("")
                out.append(SEPARATOR)
                out.append("")
        out.append(lines[i])
        i += 1
    return "\n".join(out)


def strip_noise(text):
    """Text with all blank lines and separator-comment lines removed, used
    as a safety invariant: transformation must not change anything else."""
    kept = []
    for line in text.split("\n"):
        s = line.strip()
        if s == "" or SEPARATOR_RE.match(s):
            continue
        kept.append(line)
    return "\n".join(kept)


CLANG_FORMAT_BIN = "/usr/bin/clang-format"


def run_real_clang_format(args, stdin_bytes):
    import subprocess

    forwarded = [a for a in args if a != "-output-replacements-xml"]
    proc = subprocess.run(
        [CLANG_FORMAT_BIN] + forwarded,
        input=stdin_bytes,
        stdout=subprocess.PIPE,
        check=True,
    )
    return proc.stdout


def emit_replacements_xml(original_text, final_text):
    from difflib import SequenceMatcher
    from xml.sax.saxutils import escape

    orig_lines = original_text.splitlines(keepends=True)
    final_lines = final_text.splitlines(keepends=True)
    sm = SequenceMatcher(a=orig_lines, b=final_lines, autojunk=False)

    # byte offset of the start of each original line, UTF-8 encoded
    byte_offsets = [0] * (len(orig_lines) + 1)
    for idx, ln in enumerate(orig_lines):
        byte_offsets[idx + 1] = byte_offsets[idx] + len(ln.encode("utf-8"))

    parts = ["<?xml version='1.0'?>\n<replacements xml:space='preserve' incomplete_format='false'>\n"]
    for tag, i1, i2, j1, j2 in sm.get_opcodes():
        if tag == "equal":
            continue
        offset = byte_offsets[i1]
        length = byte_offsets[i2] - byte_offsets[i1]
        text = "".join(final_lines[j1:j2])
        parts.append(
            f"<replacement offset='{offset}' length='{length}'>{escape(text)}</replacement>\n"
        )
    parts.append("</replacements>\n")
    sys.stdout.write("".join(parts))


def main():
    args = sys.argv[1:]
    xml_mode = "-output-replacements-xml" in args
    original_bytes = sys.stdin.buffer.read()
    original = original_bytes.decode("utf-8")

    clang_formatted = run_real_clang_format(args, original_bytes).decode("utf-8")

    try:
        transformed = apply_separators(clang_formatted)
        if strip_noise(transformed) != strip_noise(clang_formatted):
            # Safety net: never let a classification bug touch real code.
            transformed = clang_formatted
    except Exception:
        transformed = clang_formatted

    if xml_mode:
        emit_replacements_xml(original, transformed)
    else:
        sys.stdout.write(transformed)


if __name__ == "__main__":
    main()
