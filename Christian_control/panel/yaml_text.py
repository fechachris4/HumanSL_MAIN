"""In-place reading and editing of the panel's YAML files.

goal.yaml, planner.yaml and joint_limits.yaml are hand-written, heavily
commented files, and the comments are documentation Christian relies on. So
the panel never regenerates a file from parsed structure — it edits single
values in the file's own text, and everything it did not touch survives
byte-for-byte. This module is the one place that rule is implemented.

The subset understood here is exactly what those three files use: two-space
indents, `key: value` lines, inline `[a, b, c]` lists, `#` comments. Key
names are assumed unique within the block being searched (true of all three
files). No PyYAML — the panel is stdlib-only.

Because the indent is exactly two spaces per level, a path step matches only
the block's DIRECT children: a key at the parent's indent plus two, never a
grandchild deeper down. The first step is the same rule with no parent, so
it matches only indent 0. Without that, `(left, orientation_rpy_deg)` would
find `left.path.orientation_rpy_deg` and the panel would edit — or delete —
a key the operator never named.

One deliberate loss: an inline comment on the exact line whose value the
panel changed is dropped, because it described the old value (joint_limits
annotates radians with degrees) and keeping it would make the file lie.
Comments on their own lines always survive.
"""

import re
from typing import Any

# One nesting level, in spaces. The three files use two-space indents
# throughout, and locate() relies on that to tell a child from a grandchild.
_INDENT = 2

_KEY_RE = re.compile(r"^(\s*)([A-Za-z_]\w*):(.*)$")
_VALUE_LINE_RE = re.compile(r"^(\s*[A-Za-z_]\w*:\s*)([^#]*?)(\s*#.*)?$")


def strip_comment(line: str) -> str:
    """Remove a YAML comment. '#' only opens one at line start or after
    whitespace, so a '#' inside a quoted value is left alone."""
    if line.lstrip().startswith("#"):
        return ""
    for marker in (" #", "\t#"):
        cut = line.find(marker)
        if cut >= 0:
            line = line[:cut]
    return line


def scalar(raw: str) -> Any:
    """A value as the panel wants it: numbers as floats, [..] as lists,
    anything else as the string it is."""
    text = raw.strip()
    if text.startswith("[") and text.endswith("]"):
        return [scalar(part) for part in text[1:-1].split(",") if part.strip()]
    try:
        return float(text)
    except ValueError:
        return text


def render(value: Any) -> str:
    """A Python value as these files write it."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple)):
        return "[ " + ", ".join(render(v) for v in value) + " ]"
    if isinstance(value, float):
        return repr(value)
    return str(value)


def _key_match(line: str) -> re.Match | None:
    stripped = strip_comment(line)
    return _KEY_RE.match(stripped) if stripped.strip() else None


def _block_end(lines: list[str], start: int, indent: int) -> int:
    """Index of the first key line at `indent` or shallower — the line that
    closes the block whose children are deeper than `indent`."""
    for i in range(start, len(lines)):
        m = _key_match(lines[i])
        if m and len(m.group(1)) <= indent:
            return i
    return len(lines)


def locate(lines: list[str], path: tuple[str, ...]) -> int | None:
    """Line index of the key `path` names, or None.

    Each step searches only inside the previous key's block, and matches only
    that block's direct children — indent exactly two deeper than the parent.
    The first step starts from a notional parent at indent -2, which is the
    same rule saying a top-level key sits at indent 0.
    """
    start, parent_indent = 0, -_INDENT
    index: int | None = None
    for name in path:
        end = _block_end(lines, start, parent_indent)
        index = None
        for i in range(start, end):
            m = _key_match(lines[i])
            if not m or m.group(2) != name:
                continue
            if len(m.group(1)) == parent_indent + _INDENT:
                index, parent_indent, start = i, parent_indent + _INDENT, i + 1
                break
        if index is None:
            return None
    return index


def read_value(text: str, path: tuple[str, ...]) -> Any | None:
    lines = text.split("\n")
    i = locate(lines, path)
    if i is None:
        return None
    m = _KEY_RE.match(strip_comment(lines[i]))
    value = m.group(3).strip()
    return scalar(value) if value else None


def replace_value(text: str, path: tuple[str, ...],
                  rendered: str) -> str | None:
    lines = text.split("\n")
    i = locate(lines, path)
    if i is None:
        return None
    m = _VALUE_LINE_RE.match(lines[i])
    lines[i] = m.group(1) + rendered
    return "\n".join(lines)


def insert_key(text: str, parent: tuple[str, ...], key: str,
               rendered: str) -> str | None:
    """Append `key: rendered` at the end of `parent`'s block (top level when
    parent is empty), before any trailing blank or comment lines."""
    lines = text.split("\n")
    if parent:
        p = locate(lines, parent)
        if p is None:
            return None
        parent_indent = len(_key_match(lines[p]).group(1))
        end = _block_end(lines, p + 1, parent_indent)
        indent = parent_indent + _INDENT
    else:
        end, indent = len(lines), 0
    while end > 0 and _key_match(lines[end - 1]) is None:
        end -= 1
    lines.insert(end, (" " * indent + f"{key}: {rendered}").rstrip())
    return "\n".join(lines)


def remove_key(text: str, path: tuple[str, ...]) -> str | None:
    """Delete the key's line and every deeper-indented line under it. Comment
    lines after its last child stay — the panel cannot know whose prose they
    are, and keeping a comment is the safe direction."""
    lines = text.split("\n")
    i = locate(lines, path)
    if i is None:
        return None
    indent = len(_key_match(lines[i]).group(1))
    last = i
    for j in range(i + 1, len(lines)):
        m = _key_match(lines[j])
        if m is None:
            continue
        if len(m.group(1)) <= indent:
            break
        last = j
    del lines[i:last + 1]
    return "\n".join(lines)
