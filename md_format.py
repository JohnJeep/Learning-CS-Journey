#!/usr/bin/env python3
"""
md_format.py — Markdown typography conventions for Learning-CS-Journey.

文档写作约定 (Document conventions):
  1. CJK-ASCII spacing : one space between Chinese ideographs and ASCII letters/digits
  2. Line length       : wrap lines whose display width exceeds MAX_COLS (118)
                         Chinese/fullwidth characters count as 2 columns each.
                         Skips code blocks, headings, table rows, lines with URLs.

Usage:
    python md_format.py            # apply to all .md files
    python md_format.py --dry-run  # preview changes without writing
"""

import os
import re
import sys
import unicodedata

# ── Constants ──────────────────────────────────────────────────────────────────

ROOT     = os.path.dirname(os.path.abspath(__file__))
MAX_COLS = 118  # display columns; Chinese chars are 2 columns wide

# CJK Unified Ideographs U+4E00-U+9FFF, Extension-A U+3400-U+4DBF,
# Compatibility U+F900-U+FAFF
_CJK = '一-鿿㐀-䶿豈-﫿'

_RE_CJK_ASCII = re.compile(f'([{_CJK}])([A-Za-z0-9])')
_RE_ASCII_CJK = re.compile(f'([A-Za-z0-9])([{_CJK}])')
_RE_FENCE     = re.compile(r'^(`{3,}|~{3,})')

SKIP_DIRS = frozenset({'Blogs/source/_posts/synced', 'node_modules', '.git'})

# ── Display width ──────────────────────────────────────────────────────────────

def _char_cols(c: str) -> int:
    """Return the display column width of a single character."""
    return 2 if unicodedata.east_asian_width(c) in ('W', 'F') else 1


def display_cols(s: str) -> int:
    """Return the total display column width of a string."""
    return sum(_char_cols(c) for c in s)


def _break_index(s: str, max_cols: int) -> int:
    """Return the character index just before s would exceed max_cols columns."""
    cols = 0
    for i, c in enumerate(s):
        cw = _char_cols(c)
        if cols + cw > max_cols:
            return i
        cols += cw
    return len(s)

# ── Convention 1: CJK-ASCII spacing ───────────────────────────────────────────

def _add_spaces(segment: str) -> str:
    segment = _RE_CJK_ASCII.sub(r'\1 \2', segment)
    segment = _RE_ASCII_CJK.sub(r'\1 \2', segment)
    return segment


def apply_spacing(line: str) -> str:
    """Add CJK-ASCII spaces on a single line, preserving inline code and URLs."""
    tokens = re.split(r'(`[^`\n]+`|https?://\S+)', line)
    return ''.join(
        tok if tok.startswith('`') or tok.startswith('http')
        else _add_spaces(tok)
        for tok in tokens
    )

# ── Convention 2: Line wrapping at MAX_COLS display columns ───────────────────

def _line_parts(line: str):
    """Return (first_prefix, cont_prefix, content) for wrapping."""
    stripped = line.lstrip()
    leading  = line[:len(line) - len(stripped)]

    # List item: - / * / + / N.
    m = re.match(r'^([-*+]|\d+\.) +', stripped)
    if m:
        marker = m.group(0)
        cont   = leading + ' ' * len(marker)
        return leading + marker, cont, stripped[len(marker):]

    # Blockquote
    if stripped.startswith('> '):
        pfx = leading + '> '
        return pfx, pfx, stripped[2:]

    return leading, leading, stripped


def _wrap(line: str) -> list:
    """Wrap one line so each wrapped line fits within MAX_COLS display columns."""
    if display_cols(line) <= MAX_COLS:
        return [line]

    first_pfx, cont_pfx, content = _line_parts(line)
    pfx_cols = display_cols(first_pfx)
    cont_cols = display_cols(cont_pfx)
    result  = []
    current = first_pfx + content

    while display_cols(current) > MAX_COLS:
        cur_pfx_cols = cont_cols if result else pfx_cols

        # Find the character index where current would exceed MAX_COLS
        hard_idx = _break_index(current, MAX_COLS)

        # Prefer breaking at a space before hard_idx (English word boundary)
        segment = current[:hard_idx]
        sp = segment.rfind(' ', cur_pfx_cols + 1)
        if sp > 0 and display_cols(current[:sp]) >= MAX_COLS // 2:
            break_at = sp
        else:
            # Fall back to hard_idx — safe for CJK (any character boundary)
            break_at = hard_idx

        result.append(current[:break_at].rstrip())
        tail    = current[break_at:].lstrip(' ')
        current = cont_pfx + tail

    if current.strip():
        result.append(current)
    return result


def _should_wrap(line: str) -> bool:
    stripped = line.strip()
    if stripped.startswith('#'):                       return False  # heading
    if stripped.count('|') >= 2 and '|' in stripped:  return False  # table row
    if re.search(r'https?://', stripped):              return False  # URL present
    return True

# ── Markdown processor ─────────────────────────────────────────────────────────

def process(content: str) -> str:
    lines    = content.split('\n')
    result   = []
    in_code  = False
    in_front = False

    for i, line in enumerate(lines):
        # YAML frontmatter
        if i == 0 and line.strip() == '---':
            in_front = True
            result.append(line)
            continue
        if in_front:
            result.append(line)
            if line.strip() in ('---', '...'):
                in_front = False
            continue

        # Code fences
        m = _RE_FENCE.match(line.strip())
        if not in_code and m:
            in_code = True
            result.append(line)
            continue
        if in_code:
            if m:
                in_code = False
            result.append(line)
            continue

        # Apply spacing, then wrap if needed
        processed = apply_spacing(line)
        if display_cols(processed) > MAX_COLS and _should_wrap(processed):
            result.extend(_wrap(processed))
        else:
            result.append(processed)

    return '\n'.join(result)

# ── File walker ────────────────────────────────────────────────────────────────

def _iter_files():
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in ('node_modules', '.git')]
        rel = os.path.relpath(dirpath, ROOT)
        if any(rel.startswith(s) for s in SKIP_DIRS):
            dirnames.clear()
            continue
        for fname in filenames:
            if fname.endswith('.md'):
                yield os.path.join(dirpath, fname)


def main():
    dry_run = '--dry-run' in sys.argv
    changed = 0

    for fpath in _iter_files():
        with open(fpath, encoding='utf-8', errors='replace') as f:
            original = f.read()
        processed = process(original)
        if processed == original:
            continue
        changed += 1
        if dry_run:
            rel        = os.path.relpath(fpath, ROOT)
            orig_lines = original.split('\n')
            proc_lines = processed.split('\n')
            for j, (o, p) in enumerate(zip(orig_lines, proc_lines)):
                if o != p:
                    print(f'{rel}:{j+1}')
                    print(f'  - {repr(o[:100])}')
                    print(f'  + {repr(p[:100])}')
                    break
            delta = len(proc_lines) - len(orig_lines)
            if delta > 0:
                print(f'  (+{delta} wrapped lines)')
        else:
            with open(fpath, 'w', encoding='utf-8') as f:
                f.write(processed)

    print(f'\nFiles {"would change" if dry_run else "changed"}: {changed}')


if __name__ == '__main__':
    main()
