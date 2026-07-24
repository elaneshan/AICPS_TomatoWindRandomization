# Tomato Cluster Procedural Wind Rigging System
 
Procedural rig + collision-aware randomizer for a tomato cluster asset in
Isaac Sim / USD. Pedicels and their attached leaves rotate ("flutter") within
constrained ranges to simulate wind, while a collision system rejects poses
that would cause visible interpenetration between fruit, stems, and leaves.
The eventual goal is a synthetic dataset (RGB-D, segmentation masks, cut-point
labels) for training a perception model to guide a robotic harvesting arm
(Dobot CR3 + parallel-jaw clippers).
 
## Status
 
Core rigging + collision pipeline is validated and stable:
- Pedicel and leaf rotation controllers work (`transform.py`)
- Leaf-pedicel pairing is manually verified against the viewport, not
 auto-inferred (see `leaf_pairing_overrides` below)
- Collision checking (fruit-fruit, stem-stem, cross stem/fruit, leaf-fruit,
 leaf-stem, leaf-leaf) uses a baseline-relative proportional threshold,
 consistently applied across all five collision check functions
- Full `randomize_all` regression passes 100% (15/15 items, 0 fallbacks)
 across 5 different random seeds
Not yet built: camera rig, lighting, label export pipeline, robot/gripper
integration.
 
## Architecture
 
```
registry.py     -- discovers pedicels ("Pedicel_*") and leaves
                   ("foliage_leaf_*") in the USD stage by name convention
pivot_finder.py -- computes each pedicel's hinge point (top of segment_A's
                   bbox) and each leaf's hinge point (Object_0's world
                   translation)
rig.py          -- PlantRig: combines registry + pivot_finder into
                   PedicelRigData / LeafRigData, handles leaf-pedicel
                   pairing and rebuild/reconciliation across sessions
transform.py    -- TransformController: creates a rotation-root Xform
                   ("_Controller") above each pedicel/leaf so it can be
                   rotated around its hinge without touching the original
                   (fragile, GLTF-imported) xformOp stack
mesh_distance.py-- vertex-level min-distance utilities (numpy, no scipy dep)
                   used by all collision checks; includes a verbose variant
                   that returns closest-point provenance for debugging
collisions.py   -- CollisionChecker: per-part-type collision checks (fruit
                   vs fruit, stem vs stem, cross stem/fruit, leaf vs fruit/
                   stem, leaf vs leaf), all baseline-relative (see below)
constraints.py  -- per-axis rotation limits (pedicel default ±15.5° on Y,
                   ±5° on X/Z) and validation
randomizer.py   -- samples rotations, couples leaf flutter to its paired
                   pedicel's current rotation, runs collision checks,
                   handles fallback poses, orchestrates randomize_all
```


## Known limitations / open items
 
- Collision detection is point-cloud minimum-distance, not true mesh
 interpenetration — sufficient for this use case so far but worth knowing
 if new part types with very different geometry are added.
- No trellis/environment asset yet; `check_environment_collision` and
 `check_leaf_environment_collision` exist but are untested against real
 geometry (`fruit_tolerance`/`stem_tolerance` there are uncalibrated
 placeholders).
- Placement policy (keep rest distances above ~0.01) is a workaround, not a
 guaranteed-general fix — future cluster levels with more pedicels/leaves
 should re-run the ground-truth + regression workflow above, not assume
 past tuning transfers.
## Roadmap
 
1. Overview camera (orbit/elevation/distance/look-at randomization)
2. Lighting (dome fill + key light, intensity/color-temp randomization)
3. Label export pipeline (per-class segmentation masks, cut-point keypoint
  projected from `hinge_point`, depth render) — format TBD, should match
  whatever the perception model / robot control loop expects
4. 50-image test batch, review pose/lighting/occlusion variation
5. Robot integration once USD/clipper asset is available from collaborator
  (CR3 arm kinematics confirmed via public URDF/MJCF reference repo;
  end-effector is parallel-jaw clippers, not the dexterous hand in that
  reference repo)

