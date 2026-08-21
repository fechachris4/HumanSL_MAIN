"""Text-preserving persistence for the planner-owned static scene.

The panel validates only the scene schema, JSON/YAML value types, finiteness,
and strictly positive dimensions. Grid fit, signed-distance construction,
clearance, epsilon and trajectory acceptance remain planner responsibilities.

A save changes input to a future solve, so the caller supplies the live
``commanding`` gate. This is a panel workflow boundary, not filesystem
immutability: a direct edit can bypass it, and the runtime still rereads the
live configured planner file for later requests.
"""

from __future__ import annotations

import json
import math
import os
import re
import shutil
import stat
import tempfile
from pathlib import Path
from typing import Any, Callable

from . import paths, planner_config, yaml_text

_SCENE_PATH = ("obstacles", "scene")
_PLAIN_KEY = re.compile(r"^[A-Za-z_]\w*$")
_TOKEN = re.compile(r"^[0-9a-f]{16}$")


class _SceneError(ValueError):
    pass


def _fnv1a64(data: bytes) -> int:
    value = 1469598103934665603
    for byte in data:
        value ^= byte
        value = (value * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return value


def _format_token(value: int) -> str:
    # JSON numbers cannot preserve a general uint64. The browser therefore
    # treats this fixed-width hex spelling as an opaque change token.
    return f"{value:016x}"


def _parse_token(value: Any) -> int:
    if not isinstance(value, str) or not _TOKEN.fullmatch(value):
        raise _SceneError(
            "source_fnv1a64 must be a 16-digit lowercase hexadecimal token"
        )
    return int(value, 16)


def _strip_yaml_comment(text: str) -> str:
    quote: str | None = None
    escaped = False
    for index, char in enumerate(text):
        if quote == '"':
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == quote:
                quote = None
        elif quote == "'":
            if char == quote:
                if index + 1 < len(text) and text[index + 1] == quote:
                    continue
                quote = None
        elif char in ('"', "'"):
            quote = char
        elif char == "#" and (index == 0 or text[index - 1].isspace()):
            return text[:index]
    return text


def _mapping_line(line: str) -> tuple[int, str, str] | None:
    if not line.strip() or line.lstrip().startswith("#"):
        return None
    indent = len(line) - len(line.lstrip(" "))
    if "\t" in line[:len(line) - len(line.lstrip())]:
        raise _SceneError("obstacles.scene must use space indentation")
    content = line[indent:]
    quote: str | None = None
    escaped = False
    colon = None
    index = 0
    while index < len(content):
        char = content[index]
        if quote == '"':
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == quote:
                quote = None
        elif quote == "'":
            if char == quote and index + 1 < len(content) and content[index + 1] == quote:
                index += 1
            elif char == quote:
                quote = None
        elif char in ('"', "'"):
            quote = char
        elif char == ":":
            colon = index
            break
        index += 1
    if colon is None:
        raise _SceneError("obstacles.scene contains a line without a mapping key")
    raw_key = content[:colon].strip()
    if not raw_key:
        raise _SceneError("obstacles.scene names must be non-empty strings")
    key = _decode_string(raw_key, "obstacles.scene name")
    value = _strip_yaml_comment(content[colon + 1:]).strip()
    return indent, key, value


def _decode_string(raw: str, where: str) -> str:
    if raw.startswith('"'):
        try:
            value = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            raise _SceneError(f"{where} must be a string") from None
        if not isinstance(value, str):
            raise _SceneError(f"{where} must be a string")
        return value
    if raw.startswith("'"):
        if len(raw) < 2 or not raw.endswith("'"):
            raise _SceneError(f"{where} must be a string")
        return raw[1:-1].replace("''", "'")
    return raw


def _disk_bool(raw: str, where: str) -> bool:
    if raw == "true":
        return True
    if raw == "false":
        return False
    raise _SceneError(f"{where} must be true or false")


def _disk_number(raw: str, where: str) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError):
        raise _SceneError(f"{where} must be a number") from None
    if not math.isfinite(value):
        raise _SceneError(f"{where} must be finite")
    return value


def _disk_vector(raw: str, where: str) -> list[float]:
    if not raw.startswith("[") or not raw.endswith("]"):
        raise _SceneError(f"{where} must be a list of exactly three numbers")
    parts = [part.strip() for part in raw[1:-1].split(",")]
    if len(parts) != 3 or any(not part for part in parts):
        raise _SceneError(f"{where} must be a list of exactly three numbers")
    return [_disk_number(part, f"{where}[{index}]")
            for index, part in enumerate(parts)]


def _parse_scene(text: str) -> dict[str, dict[str, Any]]:
    lines = text.split("\n")
    scene_index = yaml_text.locate(lines, _SCENE_PATH)
    if scene_index is None:
        raise _SceneError("obstacles.scene is missing")
    scene_entry = _mapping_line(lines[scene_index])
    if scene_entry is None:
        raise _SceneError("obstacles.scene is missing")
    scene_indent, _, inline = scene_entry
    if inline == "{}":
        for line in lines[scene_index + 1:]:
            child = _mapping_line(line)
            if child is None:
                continue
            if child[0] <= scene_indent:
                break
            raise _SceneError(
                "obstacles.scene inline empty table cannot have children"
            )
        return {}
    if inline:
        raise _SceneError("obstacles.scene must be a table")

    scene: dict[str, dict[str, Any]] = {}
    index = scene_index + 1
    while index < len(lines):
        entry = _mapping_line(lines[index])
        if entry is None:
            index += 1
            continue
        indent, obstacle_id, value = entry
        if indent <= scene_indent:
            break
        if indent != scene_indent + 2:
            raise _SceneError("obstacles.scene entries must be direct children")
        location = f"obstacles.scene.{obstacle_id}"
        if not obstacle_id:
            raise _SceneError("obstacles.scene names must be non-empty strings")
        if obstacle_id in scene:
            raise _SceneError(f"{location} is duplicated")
        if value:
            raise _SceneError(f"{location} must be a table")

        fields: dict[str, str] = {}
        index += 1
        while index < len(lines):
            child = _mapping_line(lines[index])
            if child is None:
                index += 1
                continue
            child_indent, field, raw = child
            if child_indent <= scene_indent + 2:
                break
            if child_indent != scene_indent + 4:
                raise _SceneError(f"{location} fields must be direct children")
            if field in fields:
                raise _SceneError(f"{location}.{field} is duplicated")
            fields[field] = raw
            index += 1

        shape_raw = fields.get("shape")
        if shape_raw is None:
            raise _SceneError(f"{location}.shape is missing")
        shape = _decode_string(shape_raw, f"{location}.shape")
        parsed: dict[str, Any] = {
            "enabled": _disk_bool(fields.get("enabled", ""), f"{location}.enabled"),
            "shape": shape,
            "center_mount_m": _disk_vector(
                fields.get("center_mount_m", ""), f"{location}.center_mount_m"
            ),
        }
        if "radius_m" in fields:
            parsed["radius_m"] = _disk_number(fields["radius_m"], f"{location}.radius_m")
        if "height_m" in fields:
            parsed["height_m"] = _disk_number(fields["height_m"], f"{location}.height_m")
        if "half_extent_m" in fields:
            parsed["half_extent_m"] = _disk_vector(
                fields["half_extent_m"], f"{location}.half_extent_m"
            )
        unknown = set(fields) - {
            "enabled", "shape", "center_mount_m", "radius_m", "height_m",
            "half_extent_m",
        }
        for field in sorted(unknown):
            parsed[field] = fields[field]
        scene[obstacle_id] = parsed

    return _validate_scene(scene)


def _number(value: Any, where: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise _SceneError(f"{where} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise _SceneError(f"{where} must be finite")
    if positive and number <= 0.0:
        raise _SceneError(f"{where} must be strictly positive")
    return number


def _vector(value: Any, where: str, *, positive: bool = False) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise _SceneError(f"{where} must be a list of exactly three numbers")
    return [_number(item, f"{where}[{index}]", positive=positive)
            for index, item in enumerate(value)]


def _validate_scene(submitted: Any) -> dict[str, dict[str, Any]]:
    if not isinstance(submitted, dict):
        raise _SceneError("scene must be a mapping")
    if any(not isinstance(name, str) or not name for name in submitted):
        raise _SceneError("obstacles.scene names must be non-empty strings")
    normal: dict[str, dict[str, Any]] = {}
    for obstacle_id in sorted(submitted):
        if not isinstance(obstacle_id, str) or not obstacle_id:
            raise _SceneError("obstacles.scene names must be non-empty strings")
        value = submitted[obstacle_id]
        location = f"obstacles.scene.{obstacle_id}"
        if not isinstance(value, dict):
            raise _SceneError(f"{location} must be a mapping")
        shape = value.get("shape")
        if not isinstance(shape, str) or shape not in ("box", "cylinder"):
            raise _SceneError(f"{location}.shape must be box or cylinder")
        expected = ({"enabled", "shape", "center_mount_m", "half_extent_m"}
                    if shape == "box" else
                    {"enabled", "shape", "center_mount_m", "radius_m", "height_m"})
        missing = sorted(expected - set(value))
        unknown = sorted(set(value) - expected)
        if missing or unknown:
            raise _SceneError(
                f"{location} keys differ; missing = [{', '.join(missing)}], "
                f"unknown = [{', '.join(unknown)}]"
            )
        if not isinstance(value["enabled"], bool):
            raise _SceneError(f"{location}.enabled must be true or false")
        item: dict[str, Any] = {
            "enabled": value["enabled"],
            "shape": shape,
            "center_mount_m": _vector(value["center_mount_m"],
                                       f"{location}.center_mount_m"),
        }
        if shape == "box":
            item["half_extent_m"] = _vector(
                value["half_extent_m"], f"{location}.half_extent_m", positive=True
            )
        else:
            item["radius_m"] = _number(
                value["radius_m"], f"{location}.radius_m", positive=True
            )
            item["height_m"] = _number(
                value["height_m"], f"{location}.height_m", positive=True
            )
        normal[obstacle_id] = item
    return normal


def _render_number(value: float) -> str:
    return repr(float(value))


def _render_key(value: str) -> str:
    return value if _PLAIN_KEY.fullmatch(value) else json.dumps(value, ensure_ascii=False)


def _render_scene(scene: dict[str, dict[str, Any]]) -> str:
    if not scene:
        return "scene: {}"
    lines = ["scene:"]
    for obstacle_id, obstacle in scene.items():
        lines.append(f"    {_render_key(obstacle_id)}:")
        lines.append(f"      enabled: {'true' if obstacle['enabled'] else 'false'}")
        lines.append(f"      shape: {obstacle['shape']}")
        center = ", ".join(_render_number(v) for v in obstacle["center_mount_m"])
        lines.append(f"      center_mount_m: [{center}]")
        if obstacle["shape"] == "box":
            extent = ", ".join(
                _render_number(v) for v in obstacle["half_extent_m"]
            )
            lines.append(f"      half_extent_m: [{extent}]")
        else:
            lines.append(f"      radius_m: {_render_number(obstacle['radius_m'])}")
            lines.append(f"      height_m: {_render_number(obstacle['height_m'])}")
    return "\n".join(lines)


def read_scene(path: Path | None = None) -> dict[str, Any]:
    target = path or paths.PLANNER_YAML
    result: dict[str, Any] = {
        "path": str(target),
        "scene": {},
        "source_fnv1a64": None,
        "error": None,
    }
    try:
        data = target.read_bytes()
    except OSError as error:
        result["error"] = f"cannot read {target}: {error}"
        return result
    result["source_fnv1a64"] = _format_token(_fnv1a64(data))
    try:
        text = data.decode("utf-8")
        result["scene"] = _parse_scene(text)
    except (UnicodeDecodeError, _SceneError) as error:
        result["error"] = str(error)
    return result


def write_scene(
    submitted: dict[str, Any],
    source_fnv1a64: str,
    *,
    path: Path | None = None,
    commanding: Callable[[], bool],
) -> tuple[bool, dict[str, Any] | str]:
    target = path or paths.PLANNER_YAML
    if commanding():
        return False, "controller is commanding; scene save is blocked"
    current = read_scene(target)
    if current["source_fnv1a64"] is None:
        return False, str(current["error"])
    try:
        source_hash = _parse_token(source_fnv1a64)
    except _SceneError as error:
        return False, str(error)
    if source_hash != int(current["source_fnv1a64"], 16):
        return False, "planner.yaml changed on disk; reload the scene before saving"
    if current["error"]:
        return False, f"cannot replace malformed disk scene: {current['error']}"
    try:
        scene = _validate_scene(submitted)
    except _SceneError as error:
        return False, str(error)

    try:
        original = target.read_bytes()
        text = original.decode("utf-8")
    except (OSError, UnicodeDecodeError) as error:
        return False, f"cannot read {target}: {error}"
    if _fnv1a64(original) != source_hash:
        return False, "planner.yaml changed on disk; reload the scene before saving"
    candidate_text = yaml_text.replace_block(text, _SCENE_PATH, _render_scene(scene))
    if candidate_text is None:
        return False, f"obstacles.scene not found in {target.name}"

    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", dir=target.parent, prefix=f".{target.name}.",
            suffix=".tmp", delete=False
        ) as temporary:
            temporary_path = Path(temporary.name)
            temporary.write(candidate_text.encode("utf-8"))
            temporary.flush()
            os.fsync(temporary.fileno())
        os.chmod(temporary_path, stat.S_IMODE(target.stat().st_mode))

        parsed = read_scene(temporary_path)
        if parsed["error"]:
            return False, f"candidate scene is invalid: {parsed['error']}"
        if parsed["scene"] != scene:
            return False, "candidate scene did not round-trip exactly"
        missing = planner_config.missing_planner_knobs(temporary_path)
        if missing:
            return False, "planner configuration is missing: " + ", ".join(missing)
        before_knobs = planner_config.read_planner_knobs(target)
        after_knobs = planner_config.read_planner_knobs(temporary_path)
        changed_knobs = [
            name for name in before_knobs
            if before_knobs[name]["value"] != after_knobs[name]["value"]
        ]
        if changed_knobs:
            return False, "scene replacement changed planner knobs: " + ", ".join(changed_knobs)

        if commanding():
            return False, "controller is commanding; scene save is blocked"
        latest = target.read_bytes()
        if _fnv1a64(latest) != source_hash:
            return False, "planner.yaml changed on disk; reload the scene before saving"
        backup = paths.panel_backup(target)
        if not backup.exists():
            shutil.copy2(target, backup)
        os.replace(temporary_path, target)
        temporary_path = None
    except OSError as error:
        return False, f"cannot save {target}: {error}"
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass
    return True, read_scene(target)
