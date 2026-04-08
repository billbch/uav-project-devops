#!/usr/bin/env python3
"""
At image build time: inject six static buildings into PX4's default Gazebo world.

Layout: two rows facing each other across y=0 (south row y=-6, north row y=+6).
Each box is 6×6×12 m (half-extent 3 m in x/y, 6 m in z). Inner faces sit at y=±3,
so the open corridor along +X is **6 m wide** in y (suitable for a small formation).

Centers along x at 12, 24, 36 m — a short hop from spawn for missions through the gap.
"""
from __future__ import annotations

import pathlib
import re
import sys

# Box geometry (must stay in sync with llm_agent corridor AABBs).
BOX_SIZE = "6 6 12"

# (model_name, pose "x y z roll pitch yaw")
BUILDING_INSTANCES: list[tuple[str, str]] = [
    ("building_obstacle_0", "12 -6 0 0 0 0"),
    ("building_obstacle_1", "24 -6 0 0 0 0"),
    ("building_obstacle_2", "36 -6 0 0 0 0"),
    ("building_obstacle_3", "12 6 0 0 0 0"),
    ("building_obstacle_4", "24 6 0 0 0 0"),
    ("building_obstacle_5", "36 6 0 0 0 0"),
]

_MATERIALS = (
    ("0.18 0.2 0.24 1", "0.22 0.24 0.3 1"),
    ("0.2 0.18 0.22 1", "0.26 0.22 0.28 1"),
)


def _one_model(name: str, pose: str, mat_i: int) -> str:
    amb, diff = _MATERIALS[mat_i % len(_MATERIALS)]
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{BOX_SIZE}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{BOX_SIZE}</size>
            </box>
          </geometry>
          <material>
            <ambient>{amb}</ambient>
            <diffuse>{diff}</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


BUILDINGS_SDF = "".join(
    _one_model(name, pose, i) for i, (name, pose) in enumerate(BUILDING_INSTANCES)
)

# Matches legacy single model and numbered corridor models.
_BUILDING_MODEL_RE = re.compile(
    r"\s*<model\s+name=\"building_obstacle[^\"]*\">[\s\S]*?</model>\s*",
    re.MULTILINE,
)


def _strip_building_models(text: str) -> str:
    return _BUILDING_MODEL_RE.sub("", text)


def _inject_before_world_close(text: str) -> str | None:
    if "</world>" not in text:
        return None
    i = text.rindex("</world>")
    return text[:i] + BUILDINGS_SDF + "\n" + text[i:]


def main() -> int:
    worlds = pathlib.Path("/px4/Tools/simulation/gz/worlds")
    if not worlds.is_dir():
        print(f"ERROR: missing {worlds}", file=sys.stderr)
        return 1

    candidates = sorted(worlds.rglob("default.sdf"), key=lambda p: (len(p.parts), str(p)))
    if not candidates:
        print("ERROR: no default.sdf under gz/worlds", file=sys.stderr)
        return 1

    updated_any = False
    for path in candidates:
        text = path.read_text(encoding="utf-8", errors="replace")
        stripped = _strip_building_models(text)
        new_text = _inject_before_world_close(stripped)
        if new_text is None:
            continue
        if new_text == text:
            print(f"OK: corridor buildings already present in {path}")
            updated_any = True
            continue
        path.write_text(new_text, encoding="utf-8")
        print(f"OK: injected 6 corridor buildings into {path}")
        updated_any = True

    if not updated_any:
        print("ERROR: could not find </world> in any default.sdf", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
