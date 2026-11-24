---
tags: [robotics, 34753, plan]
---
# 15 Study Plan: 21 Nov → exam Tue 15 Dec 2026
Built around the **MCQ exam format** (`Exercices/Exams/MCQ Printer friendly version.pdf`). Home: [[00 Index]] · Solutions: [[12 Past Exam Solutions]] · Formulas: [[13 Formula Sheet]]

## What the exam rewards
13 questions in 5 sections. Most are **grids**: you fill a whole matrix, DH table or coefficient list. One slip costs several cells, so **speed plus a self-check** is the skill to train.

| MCQ section | Questions | What you must do fast | Chapter |
|---|---|---|---|
| 1 Position & orientation | Q1.1, Q1.2 | Current vs global composition; columns = new axes in old frame | [[02 Rotations and Homogeneous Transformations]] |
| 2 DH & IK | Q2.1–Q2.3 | Count joints/links from a picture; read a DH table off frames; 2R IK (elbow up/down) | [[03 DH Convention and Forward Kinematics]], [[04 Inverse Kinematics]] |
| 3 Velocity & trajectories | Q3.1, Q3.2 | $v=J\dot q$; quintic 3×3 solve | [[05 Velocity Kinematics and Jacobian]], [[06 Trajectory Planning]] |
| 4 Vision | Q4.1–Q4.4 | $u=\lambda x/z$ → pixels (sign of $r,c$!) → base frame via $T^b_c$ → camera moved | [[09 Sensors and Robot Vision]] |
| 5 Motor control | Q5.1, Q5.2 | Count poles; steady-state error from units + torque balance | [[08 Control]] |

Dynamics ([[07 Manipulator Dynamics]]) did not appear in the MCQ. Know it conceptually only.

## Extra practice mapped to MCQ sections
The older written exams (2016–2020, with solutions in `Exercices/Exams/Exams_from_previous_years/`) are not multiple-choice. Their sub-problems are the same skills, so use them as extra drills:

| Section | Older-exam problems | Other material |
|---|---|---|
| 1 | 2017 P2 (three frames) · 2019 P2 (rotation matrix, angular velocity) · 2020 P4.1–4.2 (task frame from 3 points; see `sol4_2020.m`) | Week 2 exercises |
| 2 | 2016 P1 (PUMA DH), P2 (DH from a matrix), P4.5 (2R IK) · 2019 P1 · 2020 P1, P2.1 | Week 3 exercises, Week 4 problems 1–2 |
| 3 | 2016 P3 (constant-jerk path), P4.2 (velocity between frames) · 2020 P2.2, P3 | Week 4, Week 6 exercises |
| 4 | 2016 P4.3–4.4 · 2019 P4 · 2020 P4.3 (stereo) | `VisionExam.pdf`, Week 8 vision exercises |
| 5 | 2016 P5 (PD + disturbance, $e_{ss}$) · 2019 P5 (motor TF, $e_{ss}$) | Week 1/7 motor exercise, Week 7 control exercises A–D |

> [!tip] Check your answers
> Every problem in this table is solved step by step, next to its official answer, in `Exam_Toolkit/Robotics_Exam_Solver_Explained.ipynb`, **Section 10 (exercise bank)**. Solve it on paper first, then compare.

---

## Phase 1: Kinematics, the biggest block (Sat 21 → Fri 27 Nov)
About 2 h/day. Project work still runs alongside.

| Day | Do | Done when |
|---|---|---|
| Sat 21 | Read [[02 Rotations and Homogeneous Transformations]]. Solve **Q1.1** by hand without looking. | All 4 columns of Q1.1 match |
| Sun 22 | **Q1.2** + 2017 P2 + 2019 P2.1. Draw the frames every time. | You can say *why* each column is what it is |
| Mon 23 | Read [[03 DH Convention and Forward Kinematics]]. 2016 P1 (PUMA) + 2016 P2. | Your PUMA table matches the solution |
| Tue 24 | **Q2.1** + **Q2.2** from the PDF (cover the solution). Then 2020 P1. | 20 DH cells in ≤ 15 min |
| Wed 25 | Read [[04 Inverse Kinematics]]. **Q2.3** + 2016 P4.5 (both elbow solutions). | Verify each answer with FK |
| Thu 26 | Week 3 exercises (2R/RRR IK, decoupling). Flashcards: intro, rotations, DH. | |
| Fri 27 | **Timed mini-mock:** MCQ Sections 1–2 in 35 min. Note every mistake. | ≥ 80 % of cells correct |

## Phase 2: Velocity, trajectories, vision (Sat 28 Nov → Thu 3 Dec)
Project deadline falls in this window, so these are lighter days (about 1.5 h).

| Day | Do | Done when |
|---|---|---|
| Sat 28 | Read [[05 Velocity Kinematics and Jacobian]]. **Q3.1** + 2016 P4.2 + 2020 P2.2. | |
| Sun 29 | Read [[06 Trajectory Planning]]. **Q3.2** by hand: set up the 3×3, solve with a calculator. | Coefficients match in ≤ 8 min |
| Mon 30 | 2016 P3 + 2020 P3 (jerk-limited paths). Re-derive cubic coefficients once. | |
| Tue 1 | Read [[09 Sensors and Robot Vision]]. **Q4.1–Q4.2**: draw the image, the $r$/$c$ axes and keyboard axes first. | |
| Wed 2 | **Q4.3–Q4.4** + `VisionExam.pdf` (both questions). | All 4 pixel pairs right |
| Thu 3 | 2016 P4.3–4.4, 2019 P4, 2020 P4.3 (stereo). | You know $z=\lambda B/(u_1-u_2)$ by heart |

## Phase 3: Control + mixed practice (Fri 4 → Wed 9 Dec)
After the project hand-in: about 3 h/day.

| Day | Do | Done when |
|---|---|---|
| Fri 4 | Read [[08 Control]] §1–6. Week 7 control exercises A1–B2 (P controller). | |
| Sat 5 | [[08 Control]] §7 (DC motor, reflected inertia). Week 1/7 motor exercise. **Q5.1**: write both TFs (with and without $L$) and count poles. | You can explain 3 vs 2 poles |
| Sun 6 | **Q5.2** + 2016 P5 + 2019 P5 ($e_{ss}$ with disturbance, final-value theorem). | Units identify every symbol |
| Mon 7 | Week 7 exercises C–D (PI/PID). Flashcards: control. | |
| Tue 8 | **Weak-spot day:** redo every question you got wrong so far (keep a list). | |
| Wed 9 | Mixed drill: one problem from each section of the older exams, 15 min each. | |

## Phase 4: Mock exams and review (Thu 10 → Mon 14 Dec)

| Day | Do |
|---|---|
| Thu 10 | **Full mock 1:** the whole MCQ PDF, cold, timed (aim ≤ 2 h), calculator only. Mark it with [[12 Past Exam Solutions]]. |
| Fri 11 | Fix mock-1 mistakes. Make your own variants: change Q1.1's angles, Q2.3's target, Q3.2's end speed, Q4.4's camera pose, then recompute. |
| Sat 12 | **Full mock 2:** [[Mock MCQ 2 (practice exam + answer key).pdf]], timed (2 h), with the notebook as in the real exam; then mark it with its answer key. Spare time: older-exam problems (2019 P1, P2, P4, P5 + 2016 P3). |
| Sun 13 | [[13 Formula Sheet]] from memory: write it out on a blank page, then compare. All [[14 Flashcards]]. |
| Mon 14 | Light: flashcards + re-read the ⚠️ warnings in every chapter. Print [[Formula Sheet.pdf]], charge the laptop, open the notebook with Wi-Fi off and Run All (must say ALL CHECKS PASSED). Sleep early. |
| **Tue 15** | **Exam.** |

---

## Exam-day checklist 🎯
- **Q1-type:** write the multiplication order *before* computing (global → left, current → right). Check $\det R=1$ and that the columns are orthonormal.
- **DH grid:** write $z_i,x_i$ of every frame in world axes first, then read the parameters. Every listed option value is usually used at least once.
- **IK:** check your angles with FK. The figure tells you elbow up or down.
- **Quintic:** $c_0,c_1,c_2$ come straight from the start conditions. Only $c_3..c_5$ need the solve.
- **Vision:** draw the $r$ and $c$ directions. Depth $z$ comes from $T^b_c$ (camera height). Convert to the base frame with $p^b=T^b_cp^c$.
- **Control:** identify each symbol by its **unit**. Steady state means $\dot\theta=0$ and di/dt = 0. No integrator means a finite non-zero error under a load.
- Eliminate options by **sign, unit and order of magnitude** before computing.
