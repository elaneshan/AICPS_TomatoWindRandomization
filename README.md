# Tomato Cluster Wind Rigging + Autonomous Harvesting Perception

A research project building synthetic training data and a robot control pipeline for autonomous tomato harvesting, using a 6-axis robot arm fitted with an adaptive gripper.

## Overview

The long-term goal is a robot that can look at a cluster of tomatoes, reason about which fruit is reachable and ripe, and grasp it, without needing a human to hand-label every possible scene it might encounter.

Training a perception model for this requires a large, varied dataset of labeled images, which is impractical to collect by hand at the scale needed. This project solves that by generating the data synthetically.

The project has two halves:

1. **Procedural simulation**: A physically-plausible, collision-aware simulation of a tomato plant cluster (fruit, stems, leaves), including wind-driven motion, used to generate large volumes of labeled synthetic training data (RGB-D + segmentation) automatically.
2. **Robot integration**: A simulated (and, in progress, real) robot arm that performs vision-guided reach-and-grasp actions. This serves two purposes:
   - Generating additional training data from the robot's actual point of view.
   - Validating a trained perception model in a closed loop against real robot motion.

## Architecture

The simulation and robot-control halves run as two separate processes, communicating over a small message-passing layer.

### Simulation Process

Runs inside the physics/rendering engine and owns:

- The 3D scene
- The procedural rigging system
- Camera capture

### Control Process

Runs standalone and owns:

- Motion planning
- Robot communication
- ROS2/MoveIt integration

It is independent of the simulation engine's own process and Python environment.

This split exists because the simulation engine and the robotics motion-planning stack have incompatible underlying dependencies (different required Python versions across the two ecosystems).

Rather than fight that incompatibility, the two are kept as independent processes that communicate over a lightweight request/response channel. This mirrors patterns recommended by the simulation engine's own vendor for exactly this kind of integration.

It also has the side benefit of mapping cleanly onto real hardware later: the control process doesn't need to know or care whether it's driving a simulated arm or a physical one.

## Typical Automated Episode

A typical automated "episode" looks like this:

1. **Randomize the simulated scene**
   - Plant pose
   - Camera framing

2. **Pick a candidate target**
   - A stem or leaf to approach

3. **Solve for a robot pose**
   - Find a pose that reaches the target.
   - Check that the pose is mechanically sound and not near a singularity.

4. **Move to a safe standoff position**
   - Position the arm near the target.
   - Allow the wrist-mounted camera to capture a clear, unoccluded view.

5. **Capture a labeled training image**
   - Capture the scene from the robot's own viewpoint.

6. **Return to a known-safe rest position**
   - Ensures data-collection cleanliness.
   - Provides physical safety around a real plant.

Every step logs its result, either success or a specific, categorized failure reason, so batches can be reviewed and debugged after the fact rather than failing silently.

## Status

Both halves are functional end-to-end.

- The simulation pipeline reliably produces labeled training data.
- The robot pipeline can autonomously:
  - Sample a target
  - Plan and execute motion toward it
  - Actuate the gripper
  - Capture an observation from the robot's own camera
- The system is currently validated in simulation, with real-hardware integration actively in progress.

## Known Open Items

- **Wrist-camera placement:** The current placement is a first-pass placeholder. It produces usable images, but the exact mount position/orientation hasn't been finalized. Another pass is needed once there's a clearer picture of what the perception model actually needs to see.

- **Startup race:** A rare startup race can occur if the simulation environment is reloaded in very quick succession. It is recoverable by a manual restart but has not yet been hardened against. This is low priority and documented for whoever picks this up next.

- **Simulation-engine bug:** A bug report was filed against Isaac Sim for a mechanical-linkage simulation defect. The issue has been worked around in this project's own code. A vendor response has come in and needs review.

- **Ongoing integration:** Real-hardware integration, expanded scene variety, and improved grasp planning are all active/ongoing work. See the [Roadmap](#roadmap).

## Tech Stack

- Physics-based 3D simulation and synthetic rendering (NVIDIA Isaac Sim)
- Procedural/parametric geometry generation
- ROS2 + MoveIt for motion planning
- 6-axis robot arm
- Adaptive gripper
- Python-based inter-process control/orchestration layer

## Roadmap

- Extend robot control to the physical robot
- Improve grasp planning and approach-pose selection
- Expand the variety of simulated scenes
- Continue research into perception-guided, force-aware manipulation of plant structures

---

> This repository is part of an active research project.
