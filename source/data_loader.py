from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
TRACKS_DIR = PROJECT_ROOT / "data" / "tracks"


@dataclass(frozen=True)
class TrackData:
    name: str
    x: list[float]
    y: list[float]
    width_right: list[float]
    width_left: list[float]
    path: Path

    @property
    def point_count(self) -> int:
        return len(self.x)


def list_track_names() -> list[str]:
    return sorted(path.stem for path in TRACKS_DIR.glob("*.csv"))


def resolve_track_path(track_name: str) -> Path:
    normalized = track_name.strip().lower()
    candidates = {path.stem.lower(): path for path in TRACKS_DIR.glob("*.csv")}
    if normalized not in candidates:
        available = ", ".join(sorted(path.stem for path in TRACKS_DIR.glob("*.csv")))
        raise ValueError(f"Unknown track '{track_name}'. Available tracks: {available}")
    return candidates[normalized]


def _normalize_header(field_name: str) -> str:
    return field_name.strip().lstrip("#").strip()


def load_track(track_name: str) -> TrackData:
    track_path = resolve_track_path(track_name)
    x: list[float] = []
    y: list[float] = []
    width_right: list[float] = []
    width_left: list[float] = []

    with track_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"Track file '{track_path}' is missing a header row.")
        reader.fieldnames = [_normalize_header(field_name) for field_name in reader.fieldnames]
        for row in reader:
            if not row:
                continue
            x.append(float(row["x_m"]))
            y.append(float(row["y_m"]))
            width_right.append(float(row["w_tr_right_m"]))
            width_left.append(float(row["w_tr_left_m"]))

    return TrackData(
        name=track_path.stem,
        x=x,
        y=y,
        width_right=width_right,
        width_left=width_left,
        path=track_path,
    )