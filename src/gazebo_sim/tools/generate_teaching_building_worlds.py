#!/usr/bin/env python3
"""Generate two deterministic 100 m x 50 m teaching-building Gazebo worlds."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORLD_DIR = ROOT / "worlds"


@dataclass(frozen=True)
class Box:
    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    color: tuple[float, float, float, float] = (0.62, 0.68, 0.76, 1.0)


def wall(name: str, x: float, y: float, sx: float, sy: float) -> Box:
    return Box(name, (x, y, 1.5), (sx, sy, 3.0))


def task_a() -> tuple[list[Box], list[list[float]]]:
    """Long central corridor with classrooms on both sides."""
    boxes = [
        Box("floor", (0.0, 0.0, -0.05), (100.0, 50.0, 0.10),
            (0.34, 0.37, 0.42, 1.0)),
        wall("outer_north", 0.0, 24.875, 100.0, 0.25),
        wall("outer_south", 0.0, -24.875, 100.0, 0.25),
        wall("outer_east", 49.875, 0.0, 0.25, 50.0),
        wall("outer_west", -49.875, 0.0, 0.25, 50.0),
    ]
    # Eight-metre wall panels leave two-metre classroom doors every 10 m.
    for side_name, y in (("north", 3.0), ("south", -3.0)):
        for index, x in enumerate(range(-45, 46, 10)):
            boxes.append(
                wall(f"corridor_{side_name}_{index}", float(x), y, 8.0, 0.25)
            )
    # Classroom partitions: six large rooms per side, each with corridor access.
    for index, x in enumerate((-45, -25, -5, 15, 35)):
        boxes.append(wall(f"class_north_{index}", x, 14.0, 0.25, 21.75))
        boxes.append(wall(f"class_south_{index}", x, -14.0, 0.25, 21.75))
    # Landmark pillars break long-corridor lidar degeneracy without blocking it.
    for index, x in enumerate((-30.0, -10.0, 10.0, 30.0)):
        boxes.append(
            Box(
                f"corridor_pillar_{index}",
                (x, 0.0, 1.5),
                (0.45, 0.45, 3.0),
                (0.86, 0.48, 0.20, 1.0),
            )
        )
    # Alternating wall-side lockers and door jambs provide frequent vertical
    # faces normal to the corridor axis.  They model ordinary teaching-building
    # furniture/door frames, remain outside the central 4.4 m passage, and are
    # visible at the quadruped lidar height.  Sparse ceiling beams alone did not
    # prevent longitudinal LIO degeneracy in the first long straight segment.
    locker_x = (-46.0, -38.0, -30.0, -22.0, -14.0, -6.0,
                2.0, 10.0, 18.0, 26.0, 34.0, 42.0)
    for index, x in enumerate(locker_x):
        y = 2.62 if index % 2 == 0 else -2.62
        boxes.append(
            Box(
                f"corridor_locker_{index}",
                (x, y, 0.90),
                (0.90, 0.50, 1.80),
                (0.22, 0.42, 0.72, 1.0),
            )
        )
    for door_index, x in enumerate(range(-40, 41, 10)):
        # Offset one jamb per doorway toward the classroom so the doorway stays
        # wider than 1.7 m and the fixture cannot trap a 0.55 m-radius robot.
        side = 1.0 if door_index % 2 == 0 else -1.0
        boxes.append(
            Box(
                f"corridor_door_jamb_{door_index}",
                (float(x) + 0.82 * side, 2.78, 1.10),
                (0.18, 0.32, 2.20),
                (0.70, 0.38, 0.18, 1.0),
            )
        )
        boxes.append(
            Box(
                f"corridor_door_jamb_south_{door_index}",
                (float(x) - 0.82 * side, -2.78, 1.10),
                (0.18, 0.32, 2.20),
                (0.70, 0.38, 0.18, 1.0),
            )
        )
    # Classrooms are not empty boxes in a real deployment.  Alternating desks
    # make every room locally non-symmetric and give the low-mounted lidar
    # x/y-normal faces even after a robot leaves the central corridor.
    for room_index, room_x in enumerate((-35.0, -15.0, 5.0, 25.0, 42.0)):
        for row_index, room_y in enumerate((8.0, 14.0, 20.0)):
            x_offset = 3.2 if (room_index + row_index) % 2 == 0 else -3.2
            for side_name, side in (("north", 1.0), ("south", -1.0)):
                boxes.append(
                    Box(
                        f"classroom_desk_{side_name}_{room_index}_{row_index}",
                        (room_x + x_offset, side * room_y, 0.40),
                        (1.40, 0.70, 0.80),
                        (0.55, 0.34, 0.18, 1.0),
                    )
                )
    # A perfectly straight corridor only constrains lateral translation well:
    # Swarm-LIO2 correctly reports longitudinal LiDAR degeneracy in that
    # geometry.  Real teaching buildings contain lintels, signs and ceiling
    # beams.  Add asymmetric overhead cross-beams so the 3-D lidar has
    # surfaces normal to the corridor direction, without changing the planar
    # traversable area or giving RACER synthetic ground obstacles.
    for index, (x, height) in enumerate(
        (
            (-42.0, 2.55),
            (-33.0, 2.70),
            (-23.0, 2.48),
            (-14.0, 2.66),
            (-5.0, 2.52),
            (4.0, 2.72),
            (13.0, 2.50),
            (23.0, 2.68),
            (32.0, 2.54),
            (42.0, 2.70),
        )
    ):
        boxes.append(
            Box(
                f"corridor_overhead_landmark_{index}",
                (x, 0.0, height),
                (0.32, 5.75, 0.34),
                (0.90, 0.68, 0.18, 1.0),
            )
        )
    spawns = [[0.0, -1.5, 0.36, 0.0], [0.0, 1.5, 0.36, 0.0]]
    return boxes, spawns


def task_b() -> tuple[list[Box], list[list[float]]]:
    """Atrium loop with side wings and several alternative routes."""
    boxes = [
        Box("floor", (0.0, 0.0, -0.05), (100.0, 50.0, 0.10),
            (0.32, 0.36, 0.40, 1.0)),
        wall("outer_north", 0.0, 24.875, 100.0, 0.25),
        wall("outer_south", 0.0, -24.875, 100.0, 0.25),
        wall("outer_east", 49.875, 0.0, 0.25, 50.0),
        wall("outer_west", -49.875, 0.0, 0.25, 50.0),
        # Rectangular atrium boundary, with a four-metre doorway on each side.
        wall("atrium_north_left", -11.0, 10.0, 18.0, 0.25),
        wall("atrium_north_right", 11.0, 10.0, 18.0, 0.25),
        wall("atrium_south_left", -11.0, -10.0, 18.0, 0.25),
        wall("atrium_south_right", 11.0, -10.0, 18.0, 0.25),
        wall("atrium_west_lower", -20.0, -6.0, 0.25, 8.0),
        wall("atrium_west_upper", -20.0, 6.0, 0.25, 8.0),
        wall("atrium_east_lower", 20.0, -6.0, 0.25, 8.0),
        wall("atrium_east_upper", 20.0, 6.0, 0.25, 8.0),
    ]
    # Classroom wings above and below the loop.
    for index, x in enumerate((-40.0, -30.0, 30.0, 40.0)):
        boxes.append(wall(f"north_wing_{index}", x, 17.5, 0.25, 14.75))
        boxes.append(wall(f"south_wing_{index}", x, -17.5, 0.25, 14.75))
    # End-wing dividers retain two open passages around the atrium.
    boxes.extend(
        [
            wall("west_mid_lower", -35.0, -6.0, 30.0, 0.25),
            wall("west_mid_upper", -35.0, 6.0, 30.0, 0.25),
            wall("east_mid_lower", 35.0, -6.0, 30.0, 0.25),
            wall("east_mid_upper", 35.0, 6.0, 30.0, 0.25),
        ]
    )
    for index, (x, y) in enumerate(
        ((-30.0, -2.5), (-30.0, 2.5), (30.0, -2.5), (30.0, 2.5))
    ):
        boxes.append(
            Box(
                f"loop_pillar_{index}",
                (x, y, 1.5),
                (0.55, 0.55, 3.0),
                (0.86, 0.48, 0.20, 1.0),
            )
        )
    # The atrium's east/west links are also long and nearly parallel.  Place
    # asymmetric wall-side cabinets along those links to maintain longitudinal
    # lidar observability while preserving both central travel lanes.
    for index, x in enumerate((-45.0, -37.0, -29.0, 29.0, 37.0, 45.0)):
        y = 5.55 if index % 2 == 0 else -5.55
        boxes.append(
            Box(
                f"atrium_link_cabinet_{index}",
                (x, y, 0.90),
                (0.90, 0.55, 1.80),
                (0.24, 0.46, 0.68, 1.0),
            )
        )
    for index, (x, y) in enumerate(
        ((-12.0, -4.0), (-5.0, 4.5), (4.0, -4.5), (12.0, 4.0))
    ):
        boxes.append(
            Box(
                f"atrium_planter_{index}",
                (x, y, 0.55),
                (1.10, 1.10, 1.10),
                (0.30, 0.58, 0.28, 1.0),
            )
        )
    # Small wall-mounted cuboids add short, asymmetric faces for 3-D scan
    # matching.  Their lower face is 1.05 m above the floor, well above the
    # 0.75 m ground-robot/RACER envelope, so they do not change the planar
    # navigable mask or narrow any doorway.  They are nevertheless visible to
    # the Mid-360's upward laser channels from the atrium and side wings.
    lidar_feature_specs = []
    for x in (-46.0, -36.0, -26.0, -16.0, -6.0,
              6.0, 16.0, 26.0, 36.0, 46.0):
        lidar_feature_specs.extend(((x, 24.525), (x, -24.525)))
    for index, x in enumerate((-48.0, -40.0, -32.0, -24.0,
                               24.0, 32.0, 40.0, 48.0)):
        lidar_feature_specs.append((x, 5.65 if index % 2 == 0 else -5.65))
    for index, x in enumerate((-18.0, -14.0, -9.0, -5.0,
                               5.0, 9.0, 14.0, 18.0)):
        lidar_feature_specs.append((x, 9.65 if index % 2 == 0 else -9.65))
    for index, y in enumerate((-8.0, -5.0, 5.0, 8.0)):
        lidar_feature_specs.append((19.65 if index % 2 == 0 else -19.65, y))
    for index, (x, y) in enumerate(lidar_feature_specs):
        boxes.append(
            Box(
                f"atrium_lidar_feature_{index}",
                (x, y, 1.35),
                (0.45, 0.45, 0.60),
                (0.92, 0.32, 0.18, 1.0),
            )
        )
    # The long east/west links of the loop have the same observability issue
    # as task A.  Uneven overhead doorway beams are realistic 3-D landmarks
    # and remain above the quadruped/RACER collision slice.
    for index, (x, y, sy, height) in enumerate(
        (
            (-42.0, 0.0, 11.75, 2.58),
            (-27.0, 0.0, 11.75, 2.72),
            (27.0, 0.0, 11.75, 2.50),
            (42.0, 0.0, 11.75, 2.66),
            (-10.0, -17.5, 14.5, 2.54),
            (10.0, 17.5, 14.5, 2.70),
            # Added 2026-08-24 (v97 analysis): each 60 m x 15 m hall had a
            # single cross beam, so a robot more than ~10 m from it saw only
            # two parallel walls inside the 10 m LiDAR range and Swarm-LIO2
            # drifted along x (bot1 diverged 7.7 m at (9.4, -15.7) while
            # physically stationary).  Uneven cross beams every ~8 m keep an
            # x-constraining 3-D landmark within range everywhere in both
            # halls.  They stay >= 2.4 m high: no change to the planar
            # navigable mask and no effect on the quadrupeds' motion.
            (-26.0, -17.5, 14.5, 2.62),
            (-18.0, -17.5, 14.5, 2.48),
            (-2.0, -17.5, 14.5, 2.66),
            (6.0, -17.5, 14.5, 2.44),
            (14.0, -17.5, 14.5, 2.74),
            (22.0, -17.5, 14.5, 2.56),
            (26.0, 17.5, 14.5, 2.60),
            (18.0, 17.5, 14.5, 2.46),
            (2.0, 17.5, 14.5, 2.68),
            (-6.0, 17.5, 14.5, 2.42),
            (-14.0, 17.5, 14.5, 2.76),
            (-22.0, 17.5, 14.5, 2.52),
        )
    ):
        boxes.append(
            Box(
                f"atrium_overhead_landmark_{index}",
                (x, y, height),
                (0.34, sy, 0.34),
                (0.90, 0.68, 0.18, 1.0),
            )
        )
    # Use the same direct-line-of-sight initialization layout as task A so a
    # single documented root-frame deployment transform serves both tasks.
    spawns = [[0.0, -1.5, 0.36, 0.0], [0.0, 1.5, 0.36, 0.0]]
    return boxes, spawns


def render_world(name: str, boxes: list[Box]) -> str:
    links = []
    for item in boxes:
        x, y, z = item.center
        sx, sy, sz = item.size
        r, g, b, a = item.color
        links.append(
            f"""      <link name='{item.name}'>
        <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
        <collision name='collision'>
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material>
            <ambient>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</diffuse>
          </material>
        </visual>
      </link>"""
        )
    return f"""<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <pose>0 0 60 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    <gravity>0 0 -9.8</gravity>
    <physics type='ode'>
      <max_step_size>0.005</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>200</real_time_update_rate>
    </physics>
    <plugin filename='ignition-gazebo-physics-system' name='gz::sim::systems::Physics'/>
    <plugin filename='ignition-gazebo-sensors-system' name='gz::sim::systems::Sensors'>
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename='ignition-gazebo-imu-system' name='gz::sim::systems::Imu'/>
    <plugin filename='ignition-gazebo-user-commands-system' name='gz::sim::systems::UserCommands'/>
    <plugin filename='ignition-gazebo-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.78 0.82 0.88 1</background>
      <shadows>true</shadows>
    </scene>
    <model name='{name}'>
      <static>true</static>
{chr(10).join(links)}
    </model>
  </world>
</sdf>
"""


def write_task(stem: str, factory) -> None:
    boxes, spawns = factory()
    WORLD_DIR.mkdir(parents=True, exist_ok=True)
    (WORLD_DIR / f"{stem}.world").write_text(
        render_world(stem, boxes), encoding="utf-8"
    )
    metadata = {
        "name": stem,
        "bounds_xy_m": [-50.0, 50.0, -25.0, 25.0],
        "spawns": spawns,
        "boxes": [asdict(item) for item in boxes],
    }
    (WORLD_DIR / f"{stem}.layout.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )


def main() -> None:
    write_task("teaching_building_corridor", task_a)
    write_task("teaching_building_atrium", task_b)


if __name__ == "__main__":
    main()
