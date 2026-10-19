# Continuous Pico trigger gripper

The supported Robotiq control is continuous, not A/B buttons.

| Input | Behavior |
| --- | --- |
| Hold right side grip | Enables Cartesian arm control and trigger-derived gripper target updates. |
| Right trigger while side grip is held | `0.0` is fully open; `1.0` is fully closed; intermediate values command proportional opening. |
| Release side grip | Retains the last gripper target. Trigger changes do not command the gripper. |
| Right thumbstick press | Manual Pico recalibration. |
| A/B | Not gripper controls. |

Before the first hardware check: clear the workspace, keep the emergency stop
reachable, keep fingers/tools/cables away from the jaws, then test trigger
positions slowly from open through intermediate openings to closed. Release grip
at an intermediate opening and verify it holds. Stop immediately on unexpected
motion, contact, oscillation, or Desk warning.
