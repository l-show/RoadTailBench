import json
import re
from dataclasses import dataclass
from pathlib import Path


RTB_ID_RE = re.compile(r"^RTB(?P<num>\d{3,})(?:[^\d].*)?\.py$", re.IGNORECASE)


@dataclass
class CodeScenario:
    scene_id: str
    number: int
    script_path: Path
    metadata_path: Path = None
    metadata: dict = None


def parse_scene_selection(value):
    if not value:
        return None
    selected = set()
    for raw in value.split(","):
        part = raw.strip().upper().replace("RTB", "")
        if not part:
            continue
        if "-" in part:
            left, right = part.split("-", 1)
            start, end = int(left), int(right.replace("RTB", ""))
            if end < start:
                start, end = end, start
            selected.update(range(start, end + 1))
        else:
            selected.add(int(part))
    return selected


def load_json_if_exists(path):
    path = Path(path)
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def discover_scenarios(scene_root, metadata_root=None, scene_selection=""):
    scene_root = Path(scene_root)
    metadata_root = Path(metadata_root) if metadata_root else None
    selected = parse_scene_selection(scene_selection)
    scenes = {}
    for path in scene_root.glob("RTB*.py"):
        match = RTB_ID_RE.match(path.name)
        if not match:
            continue
        number = int(match.group("num"))
        if selected is not None and number not in selected:
            continue
        scene_id = f"RTB{number:03d}"
        metadata_path, metadata = None, {}
        if metadata_root:
            for candidate in (metadata_root / f"{scene_id}.json", metadata_root / f"{path.stem}.json"):
                if candidate.exists():
                    metadata_path = candidate
                    metadata = load_json_if_exists(candidate)
                    break
        scenes[number] = CodeScenario(scene_id, number, path, metadata_path, metadata)
    return [scenes[k] for k in sorted(scenes)]
