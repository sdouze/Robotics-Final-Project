---
tags: [robotics, 34753, index]
aliases: [Study Book, Home]
---

# 34753 Robotics: Study Book

Built from your lectures (1–4, 6, 7, 8), lecture notes, exercises with solutions, the project intro, the vision exercises and the **Winter 2025-26 MCQ exam**. Open this folder as an Obsidian vault. Math is LaTeX (`$…$`), flashcards use `Question::Answer` (works with the *Spaced Repetition* plugin).

> [!info] Legend used throughout
> - 🟦 = taken from your slides / exercises / notes
> - 📖 = standard textbook knowledge (Spong, Hutchinson, Vidyasagar) added to fill gaps. **Not in your uploaded files**
> - 🧮 = I computed it myself (numerical checks, exam answers)
> - ⚠️ = a typo or inconsistency I found in the course material. Read these before the exam
> - 🎯 = exam tip

> [!info] About "the book"
> The textbook is now in `Book/` (*Robot Modeling and Control*, 2nd ed., Spong, Hutchinson, Vidyasagar, 2020). Parts marked 📖 follow that book. Look them up there if a slide disagrees: the slides are what the exam is written from.

> [!important] 🎯 Exam: **Tuesday 15 December 2026**, individual written **MCQ** (same format as `Exercices/Exams/MCQ Printer friendly version.pdf`). Re-exam 25 May.
> **Aids: everything except the internet** (laptop, notes and the offline notebooks are allowed).
> Follow the day-by-day plan in [[15 Study Plan]].

---

## Course facts 🟦
- **Teachers:** Matteo Fumagalli (intro, control, sensors) and Konstantinos Poulios (kinematics, trajectories, dynamics, project).
- **Grading:** group written report (project assignment, compulsory) + **individual written MCQ exam**.
- **Project:** 4-DoF Dynamixel arm, 5 parts: 1 kinematics, 2 trajectories, 3 singularities + static motor loads, 4 dynamics, 5 practical task (Matlab or Python). See [[10 Project Assignment]].
- **Textbook:** Spong et al., *Robot Modeling and Control* (1st and 2nd edition have the same relevant content).

## Map: topic → note → where it appeared in the exam

| Topic | Note | Exam (Winter 2025-26) |
|---|---|---|
| Intro, joints, workspace, configurations | [[01 Introduction to Robotics]] | (basis for Q2.1) |
| Rotations, homogeneous transforms, Euler/RPY | [[02 Rotations and Homogeneous Transformations]] | Q1.1, Q1.2 |
| DH convention, forward kinematics | [[03 DH Convention and Forward Kinematics]] | Q2.1, Q2.2 |
| Inverse kinematics (decoupling, 2R, elbow up/down) | [[04 Inverse Kinematics]] | Q2.3 |
| Angular/linear velocity, Jacobian, singularities | [[05 Velocity Kinematics and Jacobian]] | Q3.1 |
| Cubic/quintic trajectories | [[06 Trajectory Planning]] | Q3.2 |
| Euler–Lagrange dynamics | [[07 Manipulator Dynamics]] | (not in this exam, but in the project) |
| Laplace, TF, PID, DC motor, independent joint control | [[08 Control]] | Q5.1, Q5.2 |
| Sensors, camera model, stereo, visual servoing | [[09 Sensors and Robot Vision]] | Q4.1–Q4.4 (+ VisionExam) |
| Worked exercises (weeks 2, 3, 4, 6, control, vision) | [[11 Exercise Solutions]] | |
| Past exam, every question solved | [[12 Past Exam Solutions]] | |
| All formulas on one page | [[13 Formula Sheet]] | |
| Self-test cards | [[14 Flashcards]] | |
| Day-by-day plan to 15 Dec | [[15 Study Plan]] | |
| Matrix operations cheat sheet with worked examples (Greek, PDF) | ![[Matrix Operations for Robotics.pdf]] | Q1–Q5 |
| Python exam toolkit (Jupyter): change the numbers, run | `Exam_Toolkit/Robotics_Exam_Toolkit.ipynb` | Q1–Q5 |
| Python exam solver, explained step by step (Jupyter): prints every formula, substitution and check. Section 10 = **exercise bank**: every past exam (2016–2020 + MCQ), weekly exercise and lecture example solved, with the official answers | `Exam_Toolkit/Robotics_Exam_Solver_Explained.ipynb` | Q1–Q5 |
| **Practice MCQ 2**: a full mock exam in the real format (new numbers and figures, 13 questions + 3 bonus), answer key with worked solutions | [[Mock MCQ 2 (practice exam + answer key).pdf]] | Q1–Q5 |
| **Formula sheet** (English, 6 printable A4 pages, every formula with a short note on when to use it) | [[Formula Sheet.pdf]] | Q1–Q5 |

## Exam format (from the past MCQ) 🟦🧮
Five sections, **every question is multiple choice, many are "fill a grid"** (e.g. choose each column of a 4×4 matrix, or each DH parameter from a list of values):

1. Position & orientation kinematics (homogeneous matrices built from a sequence of moves; frame placement in a room)
2. DH frames & inverse kinematics (read a robot picture, count joints/links, read DH table, solve 2R IK numerically)
3. Velocity kinematics & trajectory planning (apply a given Jacobian, solve a quintic's 6 coefficients)
4. Robot vision (projection, pixel ↔ world, sequences of key positions, camera move)
5. Motor control (number of poles, steady-state error under P control + disturbance)

> [!tip] 🎯 Strategy
> - The grids are **checkable**: build the whole matrix/table, then verify with a second method (e.g. FK of your IK answer).
> - Section 5 says terms *"may not use the same symbols as in the slides"*. Always **check SI units** to identify what each parameter is.
> - Many wrong options are **sign or ordering traps** (current vs fixed axis, `Atan2(x,y)` vs `atan2(y,x)`, which pixel axis is flipped). Draw the axes before computing.
> - Use MATLAB/Python for arithmetic if allowed, but you must know *which* equation to evaluate.

## Suggested study order
1. [[02 Rotations and Homogeneous Transformations]] → [[03 DH Convention and Forward Kinematics]] → [[04 Inverse Kinematics]] (largest block of marks)
2. [[05 Velocity Kinematics and Jacobian]] → [[06 Trajectory Planning]]
3. [[09 Sensors and Robot Vision]] (formulas are few and repeat every year)
4. [[08 Control]]
5. [[07 Manipulator Dynamics]] (project + conceptual)
6. Redo [[12 Past Exam Solutions]] **without looking**, then [[14 Flashcards]].

## Schedule of the course 🟦
Intro → Kinematics I → Kinematics II & IK → Velocity & Jacobian → Project setup → Trajectory planning & dynamics → Control → Sensors → project weeks → written exam (mid-December) → re-exam (late May).
