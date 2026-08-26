# Tomato Cluster Wind Rigging + Autonomous Harvesting Perception

A research project building synthetic training data and a robot control pipeline for autonomous harvesting, using a 6-axis robot arm with an adaptive gripper.

The project combines two parts:

- **Procedural simulation** — a physically plausible, collision-aware simulation of a tomato plant cluster (fruit, stems, leaves) used to generate labeled synthetic training data for a perception model.
- **Robot integration** — a simulated and (eventually) real robot arm performing vision-guided reach-and-grasp actions, used both to collect additional training data from the robot's own viewpoint and to validate the perception model in a closed loop.

## Status

Both halves are functional. The simulation pipeline reliably generates labeled training data. The robot pipeline can autonomously sample a target, plan and execute a motion to it, actuate the gripper, and capture an observation — currently in simulation, with real-hardware integration in progress.

## Roadmap

- Extend robot control to the physical robot
- Improve grasp planning
- Expand the variety of simulated scenes
- Ongoing research into perception-guided, force-aware manipulation of plant structures

---

> This repository is part of an active research project.