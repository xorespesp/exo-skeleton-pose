# Markers Mapping

Each lower-limb joint carries one AprilTag marker.

- Family: `tagStandard41h12`
- Tag ids: `0 .. 6`
- Printable sheet: `tagStandard41h12.pdf`
- On-sheet layout: `Coordinates_tagStandard41h12.json`

> Generated from: https://shiqiliu-67.github.io/apriltag-generator/

Print the sheet, cut the tags out, and stick one on each joint per the table below. 
The black-square edge length is what the estimator needs, passed via `--tag-size` (default 0.05 m). 
Measure your printed tags and set it to match.

## Tag to joint mapping

| Tag id | Joint   | Side  |
|--------|---------|-------|
| 0      | pelvis  | root  |
| 1      | r_knee  | right |
| 2      | l_knee  | left  |
| 3      | r_ankle | right |
| 4      | l_ankle | left  |
| 5      | r_foot  | right |
| 6      | l_foot  | left  |

`pelvis` (tag 0) is the root. The rest chain off it per leg:
`pelvis -> knee -> ankle -> foot`. 
Keep the ids consistent with the mapping, otherwise the joints will be swapped.

## Tag mounting orientation

The estimator treats every leg joint as a 1-DOF leg-hinge and uses that to reject the
pose-ambiguity flip. This relies on one mounting convention:

- Stick each tag so its printed **local +X axis is aligned with that joint's flexion (leg-hinge) axis**. 
  The tag's X is the axis the joint rotates about.
- Keep the convention consistent across all tags (right-handed frame).
- Keep the **+X direction** consistent too (e.g. all pointing to the robot's left / lateral side). 
  Flip rejection alone does not care about the sign, but comparing signed left/right joint angles does.

This is a constraint on **how the tag is attached**, not on the rest pose. 
The rest pose can be captured in any neutral stance;
(e.g. with the foot already 90 degrees forward of the shank)
that offset is absorbed into the captured rest reference and does not affect the leg-hinge axis. 
Capture the rest pose (`Calibrate`) with the joints in a clean, aligned neutral stance.
