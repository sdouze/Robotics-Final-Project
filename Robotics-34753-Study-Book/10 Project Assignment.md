---
tags: [robotics, 34753, project]
---
# 10 Project Assignment (compulsory, group of 5)
Sources: Project intro slides, Lecture 1, Lecture Notes 01. Prev: [[09 Sensors and Robot Vision]] · Next: [[11 Exercise Solutions]]

> [!info] You only gave me the *intro* slides, not the assignment text or Annex C. This note covers what those slides say and how it maps to the theory.

## Basics 🟦
- **Compulsory** and part of the **final grade** (group written report + individual MCQ exam).
- Groups of **5** (enrol on DTU Learn). One report per team. State **each member's contribution** and student numbers.
- Focus: **kinematics and computer vision** (minimal control theory).
- Robot: educational **4-joint (4-DoF) DIY arm with Dynamixel servos** ("DTU CourseBot"). Needs a **real robot in the lab: book in advance**. Own arm with ≥ 4 joints allowed after agreement.
- Assignment text appears on DTU Learn in course week 2; **revisions with corrections** may follow. Hand-in in the **first week of December**.
- Support answers with **text and intermediate calculations** (principles and methods must be visible).

## Structure 🟦
| Part | Problems | Topic | Notes |
|---|---|---|---|
| 1 | 1–5 | Forward, inverse, velocity kinematics | [[03 DH Convention and Forward Kinematics]], [[04 Inverse Kinematics]], [[05 Velocity Kinematics and Jacobian]] |
| 2 | 6–7 | Trajectory planning | [[06 Trajectory Planning]] |
| 3 | 8–9 | Singularities and static motor loads | $\tau=J^T F$ in [[05 Velocity Kinematics and Jacobian]] |
| 4 | 10 | Dynamics | [[07 Manipulator Dynamics]] |
| 5 | 11 | **Practical robot task** (weighs significantly more) | [[09 Sensors and Robot Vision]] + all kinematics |

Part 5 options: **picking Smarties**, **line following**, **keyboard typing** (the exam's "type CIAO" question is a miniature of this), or **your own idea approved by the course responsible**.

## Part 5 rules 🟦
- Implement in **MATLAB or Python**. Sample code is provided **as inspiration, not as a black box**.
- **No black-box robotics frameworks**, except to **verify** your own implementation.
- **Upload a video** of the physical robot doing the task. Specify a **licence** (preferably Creative Commons) for videos and code.
- Requires self-learning: servo APIs (Dynamixel SDK) and computer vision (OpenCV).

## Dynamixel workflow (Lecture Notes 01) 🟦
1. Import the SDK. 2. Open the serial port (`/dev/ttyACM0` Linux, `COM3` Windows). 3. Set the baud rate. 4. Enable torque and write goal positions/speeds. 5. Read back joint positions/velocities (feedback). Template (motor IDs, constants, read/write helpers) is in **Annex C** of the assignment.

## How the exam relates to the project
The MCQ is individual and tests the same tools at small scale: DH frames of a picture (Q2), IK of a 2R arm (Q2.3), Jacobian/velocity (Q3.1), quintic trajectories (Q3.2), camera ↔ keyboard coordinates (Q4). Doing the project parts by hand first is the best exam practice.
