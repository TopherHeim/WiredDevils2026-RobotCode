# **WiredDevils5498 FRC Robot Code**

This repository contains the source code for the **WiredDevils5498** FRC (FIRST Robotics Competition) robot, written for the **Rebuilt 2026** season. The code is written in Java and runs on a RoboRIO. It includes functionality for the robot's swerve drivetrain, elevator, wrist, intake, shooter, and autonomous path following.

---

## **Table of Contents**

- [Robot Overview](#robot-overview)
- [Subsystems](#subsystems)
- [Features](#features)
- [Turret Auto-Aim](#turret-auto-aim)
- [Controls](#controls)
- [Autonomous Named Commands](#autonomous-named-commands)
- [Setting Constants](#setting-constants)
- [How to Use](#how-to-use)
- [License](#license)

---

## **Robot Overview**

The **WiredDevils5498** robot features a **swerve drive system** that allows precise omnidirectional movement and independent rotation. For Rebuilt 2026, the robot is designed to score coral game pieces at multiple reef levels using an elevator, a dedicated wrist subsystem, and a shooter. A ground intake handles game piece collection, and a shooter lookup table maps distance to shooter velocity for automated shooting.

---

## **Subsystems**

- **Swerve Drive**: Full omnidirectional movement with field-relative control, heading lock, speed dampening, and turret auto-aim.
- **Elevator**: Multi-stage elevator with encoder-based setpoints for L1, L2, L3, algae height, and player station height.
- **Wrist**: Dedicated wrist subsystem (separate from the elevator), controlled via a driver button or preset positions from the upper controller. Supports a secondary "DriveToApril" wrist position mode.
- **Intake**: Ground intake mechanism driven by the driver's left trigger.
- **Shooter**: Handles coral game piece scoring. Supports a distance-based lookup table (`ShooterLookup`) for velocity targeting. SysId characterization routines are available on SmartDashboard.
- **Motor**: Auxiliary motor subsystem toggled by the driver's right bumper, coordinated with the shooter.
- **Actuator2**: Linear actuator extended by the driver's B button. (Retract currently inactive.)
- **Climber**: Subsystem present in code but currently inactive (commented out).
- **Pose Estimator**: Tracks robot position on the field using `SwerveDrivePoseEstimator`.
- **ShooterLookup**: Lookup table mapping distance to shooter speed/percent output for range-based shooting.

---

## **Features**

- **Autonomous Mode**: Path following via **PathPlannerLib** with named commands for elevator heights, wrist angles, shooter actions, and coral intake detection.
- **Teleoperated Mode**: Full manual control via two Xbox controllers (driver + upper operator).
- **Heading Lock**: POV buttons snap and hold the robot to cardinal headings (0°, 90°, 180°, 270°).
- **Turret Auto-Aim**: Holding Y on the driver controller auto-aims the drivetrain toward a target while still allowing full translation input. Also drives the shooter forward via right trigger while held.
- **Speed Dampening**: Left Bumper on the driver controller reduces drive speed for precise maneuvering.
- **Shooter SysId**: Four SysId characterization routines (Quasistatic Forward/Reverse, Dynamic Forward/Reverse) are exposed on SmartDashboard for shooter motor tuning.
- **Auto Chooser**: Autonomous routines selectable from SmartDashboard via `AutoBuilder`.

---

## **Turret Auto-Aim**

`AimTurretCommand` is activated by holding the **Y button** on the driver controller. While active, it continuously rotates the robot to face a fixed field target (the hub/speaker) while still allowing the driver to translate freely. It also sets the shooter to the correct RPM when the right trigger is pressed.

### How It Works

**1. Target Geometry**

The command calculates the vector from the robot's current AprilTag odometry position to a hardcoded field target coordinate (`x = 4.611624 m, y = 4.021328 m`). This coordinate must be updated per alliance (Blue vs. Red) before competition.

**2. Ballistic Trajectory Solving**

Rather than using a simple distance-to-RPM lookup, this command solves for the true ballistic flight time accounting for robot velocity at the moment of firing. This means the robot leads its shot, it aims where the game piece needs to go, not just where the target is.

The solver uses **Newton's method** to find the flight time `t` that satisfies the constraint that a projectile launched at a fixed angle (currently `50°`) will reach the target height (`1.8288 m`) given the horizontal distance, robot velocity components (`Vx`, `Vy`), and gravity. Newton's method iterates up to 20 times; if it does not converge, the shot is aborted.

From the solved flight time, the required launch speed is back-calculated, and then converted to a shooter RPM via a linear calibration equation.

**3. Rotation Control**

The required traverse angle (direction to lead the shot) is computed via `atan2` using the velocity-compensated target vector. A **PID controller** (tuned via `TurretConstants`) computes the rotation command, clamped to a max output, and drives the swerve accordingly.

**4. SmartDashboard Telemetry**

While active, the following values are published to SmartDashboard for tuning and debugging:

| Key | Description |
|---|---|
| `errorDeg` | Heading error to target in degrees |
| `LaunchSpeed` | Calculated required launch speed (m/s) |
| `FlightTime` | Solved projectile flight time (s) |
| `TraverseAngle` | Velocity-compensated aim angle (degrees) |
| `Robot Yaw` | Current robot heading (degrees) |
| `RPM` | Commanded shooter RPM |
| `dx` / `dy` | Distance components to target (m) |
| `Vx` / `Vy` | Robot field-relative velocity components (m/s) |
| `Total Dist` | Straight-line distance to target (m) |

### Tuning Notes

- **Launch angle** (`launchAngleDeg`, currently `50°`): Adjust based on static shooting tests. A steeper angle increases hang time but reduces range; a shallower angle does the opposite.
- **Launch height** (`launchHeight`, `0.457 m`) and **hub height** (`hubHeight`, `1.8288 m`): Update these to match your robot's actual shooter exit point and the target height.
- **RPM conversion** (`distance_conv` and `RPM` equations): These are linear calibration fits and will need to be re-derived if the shooter changes. Use SysId data and static shot testing to re-fit.
- **Minimum distance**: The solver skips shots closer than `2 m` to the target ("Too close to find solution"). Adjust this threshold if needed.
- **`TurretConstants.kP / kI / kD`**: Tune the rotation PID so the robot snaps to the target heading quickly without oscillating.
- **Alliance coordinate**: The hub coordinates are currently set for one alliance. Remember to flip the target coordinates for the opposite alliance before matches.

---

## **Controls**

### **Driver Controls (Joystick 0 - Xbox Controller)**

| Input | Action |
|---|---|
| Left Stick Y-Axis | Drive forward / backward |
| Left Stick X-Axis | Strafe left / right |
| Right Stick X-Axis | Rotate robot |
| Left Trigger | Run intake |
| Right Trigger | Shooter forward (also used during auto-aim) |
| Y Button | Turret auto-aim (hold) |
| A Button | DriveToApril / Wrist DriveToApril mode (hold) |
| X Button | Wrist move toggle |
| B Button | Extend actuator |
| Left Bumper | Dampen drive speed |
| Right Bumper | Auxiliary motor toggle (with shooter) |
| POV Up (90°) | Lock heading to 90° |
| POV Down (270°) | Lock heading to 270° |
| POV Left (180°) | Lock heading to 180° |
| POV Right (0°) | Lock heading to 0° |

> **Note:** Gyro is zeroed automatically on robot startup. There is no manual gyro reset button in the current build.

## **Autonomous Named Commands**

The following named commands are registered for use in PathPlanner autos:

| Command Name | Action |
|---|---|
| `ElevatorLevel3` | Move elevator to L3 scoring height |
| `ElevatorLevel2` | Move elevator to L2 scoring height |
| `ElevatorLevel1` | Move elevator to L1 scoring height |
| `Algae` | Move elevator to algae collection height |
| `ElevatorPlayerStation` | Move elevator to player station pickup height |
| `WristLevel1` | Set wrist angle for L1 |
| `WristLevel2` | Set wrist angle for L2 |
| `WristLevel3` | Set wrist angle for L3 |
| `WristPlayerStation` | Set wrist angle for player station |
| `WaitForCoral` | Wait until a coral game piece is detected by the shooter sensor |
| `Shoot` | Run shooter to score |
| `autonCommand` | Run custom auton command sequence |

---

## **Setting Constants**

All distance units must be in **meters** and rotation units in **radians**. Adjust the following in `Constants.java`:

1. **Gyro**: Set `pigeonID` and `invertGyro`. Gyro rotation must be CCW+ (Counter Clockwise Positive).
2. **`chosenModule`**: If using a supported COTS SDS module, set this to automatically configure wheel circumference, gear ratios, motor inverts, and angle PID values. If not using a supported module, delete this variable and manually set those constants.
3. **`trackWidth`**: Center-to-center distance of left and right swerve modules (meters).
4. **`wheelBase`**: Center-to-center distance of front and rear modules (meters).
5. **`wheelCircumference`**: Circumference of the wheel including tread (meters). Auto-set for supported modules.
6. **`driveGearRatio`** and **`angleGearRatio`**: Auto-set for supported modules.
7. **`canCoderInvert`** and **`angleMotorInvert`**: Both must be CCW+. Auto-set for supported modules.
8. **`driveMotorInvert`**: Auto-set for supported modules. Can remain false if offsets are set correctly.
9. **Module-Specific Constants**: Set CAN IDs for each module's drive motor, angle motor, and CANCoder.
10. **Setting Offsets**:
    - Use a straight piece of 1x1 metal to align all modules forward.
    - Point bevel gears the same direction so a positive drive input moves the robot forward (verify with Phoenix Tuner).
    - Read the four "Mod X Cancoder" values from SmartDashboard and copy them (to 2 decimal places) into each module's `angleOffset` using `Rotation2d.fromDegrees(value)`.
11. **Angle Motor PID**: Auto-set for supported modules. To tune manually: start P at 0.01, double until oscillation, then binary search down. Add D only if there is overshoot. Leave I at 0.
12. **`maxSpeed`** (m/s) and **`maxAngularVelocity`** (rad/s): Use theoretical values or measure from the real robot.
13. **Drive Characterization (KS, KV, KA)**: Use the WPILib characterization tool with modules locked straight forward, treating it as a tank drive. For the **shooter**, use the SysId routines exposed on SmartDashboard.
14. **`driveKP`**: Tune after inserting characterization values. Increase until there is no overshoot or oscillation. Leave `driveKI`, `driveKD`, and `driveKF` at 0.

---

## **How to Use**

1. **Clone the repository** to your local machine.
2. **Build the project** using Gradle: `./gradlew build`
3. **Deploy** the code to the RoboRIO using WPILib's tools.
4. Select your autonomous routine from the **SmartDashboard Auto Chooser** before the match.
5. **Control the robot** using the driver and upper operator controllers during teleop.

---

## **License**

Copyright (c) 2009-2024 FIRST and other WPILib contributors. All rights reserved — see the [LICENSE.md](WPILib-License.md) file for details.
