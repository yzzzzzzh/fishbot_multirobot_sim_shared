#!/usr/bin/env python3
"""
Deterministic maze generator.

Produces, from a single source of truth:
  * a Gazebo SDF world  -> ../worlds/fishbot_maze.world
  * a machine-readable definition (walls / bounds / spawns / goals)
    -> ./maze_def.json     (consumed by the planner & controller)

The maze is a recursive-backtracker perfect maze, then "braided" (some extra
walls removed) to create loops, with a 3x3 open chamber carved at the centre so
the 5 robots can start clustered and see each other (good for Swarm-LIO2's
teammate initialisation). Corridors are PITCH-THICK wide (~1.4 m).
"""
import argparse
import json
import os
import random

# --- layout parameters --------------------------------------------------------
NC, NR = 7, 7          # cells per axis
PITCH = 1.6            # cell pitch [m]  -> corridor width ~1.4 m
THICK = 0.2           # wall thickness [m]
HEIGHT = 1.0          # wall height [m] (default; override with --height)
SEED = 7
CHAMBER = {2, 3, 4}   # central cells opened into one chamber

X0 = -(NC - 1) / 2 * PITCH
Y0 = -(NR - 1) / 2 * PITCH
HALF = (NC - 1) / 2 * PITCH + PITCH / 2   # outer wall offset

HERE = os.path.dirname(os.path.abspath(__file__))
WORLD_PATH = os.path.normpath(os.path.join(HERE, "..", "worlds", "fishbot_maze.world"))
DEF_PATH = os.path.join(HERE, "maze_def.json")


def cell_center(i, j):
    return (X0 + i * PITCH, Y0 + j * PITCH)


def carve_maze():
    # walls_v[i][j]: wall between cell (i,j) and (i+1,j),  i in [0,NC-2]
    # walls_h[i][j]: wall between cell (i,j) and (i,j+1),  j in [0,NR-2]
    walls_v = [[True] * NR for _ in range(NC - 1)]
    walls_h = [[True] * (NR - 1) for _ in range(NC)]
    rng = random.Random(SEED)

    visited = [[False] * NR for _ in range(NC)]
    stack = [(0, 0)]
    visited[0][0] = True
    while stack:
        i, j = stack[-1]
        nbrs = []
        if i > 0 and not visited[i - 1][j]:
            nbrs.append((i - 1, j, "L"))
        if i < NC - 1 and not visited[i + 1][j]:
            nbrs.append((i + 1, j, "R"))
        if j > 0 and not visited[i][j - 1]:
            nbrs.append((i, j - 1, "D"))
        if j < NR - 1 and not visited[i][j + 1]:
            nbrs.append((i, j + 1, "U"))
        if not nbrs:
            stack.pop()
            continue
        ni, nj, d = rng.choice(nbrs)
        if d == "R":
            walls_v[i][j] = False
        elif d == "L":
            walls_v[i - 1][j] = False
        elif d == "U":
            walls_h[i][j] = False
        else:
            walls_h[i][j - 1] = False
        visited[ni][nj] = True
        stack.append((ni, nj))

    # braid: remove ~1/3 of remaining internal walls to create loops
    removable = ([("v", i, j) for i in range(NC - 1) for j in range(NR) if walls_v[i][j]]
                 + [("h", i, j) for i in range(NC) for j in range(NR - 1) if walls_h[i][j]])
    rng.shuffle(removable)
    for kind, i, j in removable[: len(removable) // 3]:
        if kind == "v":
            walls_v[i][j] = False
        else:
            walls_h[i][j] = False

    # open the central chamber (remove every internal wall among CHAMBER cells)
    for i in range(NC - 1):
        for j in range(NR):
            if i in CHAMBER and (i + 1) in CHAMBER and j in CHAMBER:
                walls_v[i][j] = False
    for i in range(NC):
        for j in range(NR - 1):
            if i in CHAMBER and j in CHAMBER and (j + 1) in CHAMBER:
                walls_h[i][j] = False

    return walls_v, walls_h


def build_boxes(walls_v, walls_h):
    """Return list of (cx, cy, sx, sy) axis-aligned wall boxes."""
    boxes = []
    # internal vertical walls
    for i in range(NC - 1):
        for j in range(NR):
            if walls_v[i][j]:
                cx = X0 + (i + 0.5) * PITCH
                cy = Y0 + j * PITCH
                boxes.append((cx, cy, THICK, PITCH + THICK))
    # internal horizontal walls
    for i in range(NC):
        for j in range(NR - 1):
            if walls_h[i][j]:
                cx = X0 + i * PITCH
                cy = Y0 + (j + 0.5) * PITCH
                boxes.append((cx, cy, PITCH + THICK, THICK))
    # outer border
    span = NR * PITCH + THICK
    boxes.append((0, HALF, NC * PITCH + THICK, THICK))
    boxes.append((0, -HALF, NC * PITCH + THICK, THICK))
    boxes.append((HALF, 0, THICK, span))
    boxes.append((-HALF, 0, THICK, span))
    return boxes


def write_world(boxes):
    links = []
    for n, (cx, cy, sx, sy) in enumerate(boxes):
        links.append(f"""
      <link name='wall_{n}'>
        <pose>{cx:.4f} {cy:.4f} {HEIGHT/2:.3f} 0 0 0</pose>
        <collision name='c'><geometry><box><size>{sx:.4f} {sy:.4f} {HEIGHT:.3f}</size></box></geometry></collision>
        <visual name='v'><geometry><box><size>{sx:.4f} {sy:.4f} {HEIGHT:.3f}</size></box></geometry>
          <material><ambient>0.25 0.35 0.6 1</ambient><diffuse>0.3 0.45 0.75 1</diffuse></material>
        </visual>
      </link>""")
    world = f"""<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name='ground_plane'><static>1</static><link name='link'>
      <collision name='collision'><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
      <visual name='visual'><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
    </link></model>
    <gravity>0 0 -9.8</gravity>
    <physics type='ode'><max_step_size>0.004</max_step_size><real_time_factor>1</real_time_factor><real_time_update_rate>250</real_time_update_rate></physics>
    <plugin filename="ignition-gazebo-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="ignition-gazebo-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="ignition-gazebo-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <scene><ambient>0.4 0.4 0.4 1</ambient><background>0.7 0.7 0.7 1</background><shadows>1</shadows></scene>
    <model name='maze'><static>1</static><pose>0 0 0 0 0 0</pose>{''.join(links)}
    </model>
  </world>
</sdf>
"""
    os.makedirs(os.path.dirname(WORLD_PATH), exist_ok=True)
    with open(WORLD_PATH, "w") as f:
        f.write(world)
    return len(boxes)


def main():
    # --height/--out let the drone scenario generate a tall-wall variant of the
    # SAME maze (same SEED -> same layout) into a separate world file, leaving
    # fishbot_maze.world and maze_def.json byte-identical for the 2D scenarios.
    global HEIGHT, WORLD_PATH
    ap = argparse.ArgumentParser()
    ap.add_argument("--height", type=float, default=HEIGHT, help="wall height [m]")
    ap.add_argument("--out", default=None,
                    help="world filename in ../worlds/ (default fishbot_maze.world)")
    args = ap.parse_args()
    HEIGHT = args.height
    custom_out = args.out is not None
    if custom_out:
        WORLD_PATH = os.path.normpath(os.path.join(HERE, "..", "worlds", args.out))

    walls_v, walls_h = carve_maze()
    boxes = build_boxes(walls_v, walls_h)
    n = write_world(boxes)

    # spawn circle (must match the launch: pattern=circle, base=(0,0), r, count)
    import math
    count, radius = 5, 0.7
    spawns = {}
    for k in range(count):
        ang = 2 * math.pi * k / count
        spawns[f"bot{k+1}"] = [radius * math.cos(ang), radius * math.sin(ang), ang]

    if custom_out:
        # layout is identical (same SEED); don't touch the 2D scenarios' def file
        print(f"wrote {WORLD_PATH} ({n} wall boxes, height {HEIGHT} m)")
        print("maze_def.json left untouched (custom --out)")
        return

    defn = {
        "nc": NC, "nr": NR, "pitch": PITCH, "thick": THICK,
        "x0": X0, "y0": Y0, "half": HALF,
        "bounds": [-HALF - 0.4, HALF + 0.4, -HALF - 0.4, HALF + 0.4],
        "boxes": boxes,                 # (cx,cy,sx,sy)
        "spawns": spawns,               # bot -> [x,y,yaw]
        "spawn_center": [0.0, 0.0], "spawn_radius": radius, "count": count,
        "chamber_cells": sorted(CHAMBER),
        "corner_cells": {"bl": [1, 1], "br": [NC - 2, 1],
                         "tl": [1, NR - 2], "tr": [NC - 2, NR - 2]},
    }
    with open(DEF_PATH, "w") as f:
        json.dump(defn, f, indent=2)
    print(f"wrote {WORLD_PATH} ({n} wall boxes)")
    print(f"wrote {DEF_PATH}")
    print(f"arena +/-{HALF:.2f} m, corridor ~{PITCH-THICK:.2f} m, spawns: "
          + ", ".join(f"{b}=({p[0]:.2f},{p[1]:.2f})" for b, p in spawns.items()))


if __name__ == "__main__":
    main()
