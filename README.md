# mimic controllers

requires:
- [subscription_notifier](https://github.com/CNR-STIIMA-IRAS/subscription_notifier)
- [name_sorting](https://github.com/CNR-STIIMA-IRAS/name_sorting)

## Controllers

### Mimic controller:
control multiple joints, each one has a (position+velocity) setpoint which proportionally depends on a leading one.

required parameters:
``` yaml
mimic_controller:
  type: robot_control/MimicController
  joint_names: # controlled joints
  - jnt1
  - jnt2
  - jnt3

  leading_joint: "jnt1"  # leading joint

  # target position of a generic joint:  offset+multiplier*position of leading joint
  # target velocity of a generic joint:  multiplier*velocity of leading joint
  multiplier: [1.0, -1.0, 1.0] #  (default 1.0)
  offset: [0.0, 0.0, 0.0] # (default null)

  Kp: [1.0, 2.0, 3.0] # position gain of controller
  Kv: [1.0, 2.0, 3.0] # velocity gain of the controller (default null)
  Ki: [1.0, 2.0, 3.0] # integral gain of the controller (default null)
  max_effort: [50.0, 50.0, 50.0] # maximum effort of the joint
  setpoint_topic_name: "/manipulator/joint_target"

```

### Mimic effort controller
control multiple joints, each one has a (effort) setpoint which proportionally depends on a leading one.

``` yaml
mimic_effort_controller:
  type: robot_control/MimicEffortController
  joint_names: # controlled joints
  - jnt1
  - jnt2
  - jnt3

  leading_joint: "jnt1"  # leading joint

  # target effort of a generic joint:  offset+multiplier*effort of leading joint

  multiplier: [1.0, -1.0, 1.0] #  (default 1.0)
  offset: [0.0, 0.0, 0.0] # (default null)

  setpoint_topic_name: "/manipulator/joint_target"

```


### Two finger controller
Control two joints (typically the two finger of a gripper). The effort setpoint is specified on the leading joint. A spring-damper controller try to keep the position of the two joints equal.

```yaml
gripper_controller:
  type: robot_control/TwoFingersController
  leading_joint: "right_finger_joint"
  following_joint: "left_finger_joint"
  spring: 10  # elasticity between finger: add an effort contribution spring*(leading_joint_position-following_joint_position)
  damper: 10  # viscosity between finger: add an effort contribution spring*(leading_joint_velocity-following_joint_velocity)
  setpoint_topic_name: "/gripper/joint_target"
```
