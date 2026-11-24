---
tags: [robotics, 34753, intro]
---
# 01 Introduction to Robotics
Sources: Lecture 1, Lecture Notes 01, Lecture 2 (first half). Next: [[02 Rotations and Homogeneous Transformations]] · Back: [[DTU/Semester 1/Robotics/Robotics-34753-Study-Book/00 Index]]

## What is a robot? 🟦
- Word *robota* (Czech: forced labour), from Karel Čapek's 1920 play *R.U.R.*
- RIA (Robotics Institute of America): *a reprogrammable, multifunctional manipulator designed to move materials, parts, tools or specialised devices through variable programmed motions to perform a variety of tasks.*
- European definition: *an independent acting and self-controlling machine, equipped with specific tools to handle or machine, whose movements are programmable.*
- Working definition: a **programmable machine that senses, plans and acts** in the physical world.

**Asimov's three laws**
1. Do not injure a human (or allow harm through inaction).
2. Obey humans unless it conflicts with law 1.
3. Protect own existence unless it conflicts with laws 1–2.

**Types of robots:** manipulators · wheeled mobile · legged (BigDog) · humanoid · aerial · underwater.
**Why use robots:** quality (accuracy, repeatability), efficiency (no fatigue/vacation), safety (dangerous environments, e.g. nuclear decontamination).

## The operational loop 🟦
$$\text{Sensing} \rightarrow \text{Planning} \rightarrow \text{Actuation} \rightarrow \text{Feedback} \;(\text{closes the loop})$$
The course follows the *analytical pipeline*: **Kinematics → Path/Trajectory planning → Dynamics → Control → Perception (vision)**.

## Anatomy of a serial arm 🟦
- **Base (link 0, fixed)**, **links** (rigid bodies, the "bones"), **joints** (the "hinges"), **end-effector / TCP** (Tool Center Point: gripper, torch, camera).
- A serial manipulator with $n$ joints has **$n$ moving links + 1 fixed link = $n+1$ links** and $n+1$ frames.
- One **joint variable** per joint: $\theta_i$ (revolute, rad/deg) or $d_i$ (prismatic, mm).
- **Joint $i$ is fixed to link $i-1$ and moves link $i$**. (Often asked in exam questions: "which joint does this motor drive?")

### Joint types
| | Revolute (R) | Prismatic (P) |
|---|---|---|
| DoF / variable | 1 / $\theta$ | 1 / $d$ |
| Pros | compact, simple, even transmission of constraint forces, easy to lubricate, insensitive to dirt | produces linear motion directly |
| Cons | hard to produce linear motion in limited space | larger, more complex, uneven reaction forces, hard to lubricate, dirt-sensitive |
| Reach | covers twice its link length | covers only its link length |

A **spherical joint** (3 DoF) is *not considered* in the course. A **spherical wrist** has 3 revolute axes (roll, pitch, yaw) intersecting in one point, the **wrist centre**.

## Degrees of freedom, configuration, workspace 🟦
- **DoF** = number of independent parameters that define the configuration. Free rigid body: 6 (x, y, z + roll, pitch, yaw).
- General positioning + orienting needs **6 DoF**; a rotationally symmetric tool (welding, grinding, polishing) needs **5**; pick-and-place on a plane needs **4** (X, Y, Z, roll).
- **Configuration space** = all configurations; its dimension = DoF.
- **Redundancy:** $n > m$ (more joints than task dimensions).
- **Reachable workspace:** all points the end-effector can reach. **Dexterous workspace:** points reachable with *arbitrary* orientation. (Dexterous ⊂ reachable.)
- Choosing a robot: workspace (reach all fixtures, free air around them, mind singularities), load capacity, speed, repeatability & accuracy, external interfaces.

## Kinematic configurations 🟦
| Type | Joints | Notes |
|---|---|---|
| Articulated (elbow, anthropomorphic) | RRR | $z_1 \parallel z_2$, $z_1 \perp z_0$; largest workspace for its size |
| Spherical | RRP | joint coords = spherical coords of the end-effector; axes meet at a point |
| SCARA | RRP | $z_0 \parallel z_1 \parallel z_2$; table-top assembly, pick-and-place. *Selective Compliant Articulated Robot for Assembly* |
| Cylindrical | RPP | material transfer |
| Cartesian | PPP | simplest kinematics; ⚠️ the slide title says "RPP" but a Cartesian robot is **PPP** |

The *type of the joints before the wrist* defines the configuration. **Parallel manipulators** contain closed chains (≥2 chains from base to end-effector): more rigid and accurate, fundamentally different from serial arms.

**Grippers:** two-finger parallel jaw, parallelogram, adaptive, scissor; three-finger / anthropomorphic hands give more dexterity.

## Architecture of a robotic system 🟦
Mechanical structure (kinematic + dynamic model) · actuators (electrical, hydraulic, pneumatic, artificial muscle) · computation/controllers · sensors (environmental and configuration sensors) · communications · user interface. Interdisciplinary: mechanical engineering, electrical engineering, computer science, cognitive psychology/perception.

## Tools (Lecture Notes 01) 🟦
- **MATLAB:** matrix-oriented; Simulink for block diagrams; Control System Toolbox; Symbolic Math Toolbox; talks to Dynamixel servos through the Dynamixel SDK.
- **Python:** NumPy (arrays), SymPy (symbolic), Matplotlib, OpenCV (vision), Dynamixel SDK.
- Dynamixel setup steps: import SDK → open serial port (`/dev/ttyACM0` or `COM3`) → set baud rate → enable torque and send commands.

## Quick self-check
- A 4-DoF SCARA arm: joints R, R, P, R. How many links and frames? → 5 links (incl. base), 5 frames.
- Pick-and-place on a table needs how many DoF? → 4.
