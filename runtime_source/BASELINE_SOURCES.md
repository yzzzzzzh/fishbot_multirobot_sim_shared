# Sources for the frozen first-success single-Go2 baseline

This directory contains source snapshots extracted from the exact local images
used by `run_first_success_baseline.sh`.  It is documentation/source release
material only: the frozen launcher continues to use the image IDs and the
host-mounted code/configuration at the repository root.

| Runtime component | Image ID | Extracted source |
| --- | --- | --- |
| RACER ROS 1 | `sha256:c070b48a127a615497d3ba5e8c7bd6809daf7221e2b4877621cb08d9f82924b2` | `racer_ros1/RACER/` |
| Swarm-LIO2 ROS 2 | `sha256:28d6c99f46b8a4bc38c4fa2b3bd6c0dfc0eb1efb744921851c1bbb9b08e259b5` | `swarm_lio2_ros2/` |
| Go2 legged runtime | `sha256:d7d348ff3aca68ca87e2b315e85f508a97b1de26c9cc70ac9949a3947cf454d9` | `legged/` |
| Gazebo runtime | `sha256:b203b7bc272af9f0c98b343aa6a2ffb47c607370f2e7bc29aaf12d9f363b3db1` | `gazebo/` |

Excluded from this release: build/install/log directories, generated runtime
output, caches, Docker images, and learned policy weights.  The baseline's
runtime-mounted files and parameters remain unchanged at the repository root.
