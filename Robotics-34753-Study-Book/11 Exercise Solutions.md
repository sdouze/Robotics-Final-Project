---
tags: [robotics, 34753, exercises, solutions]
---
# 11 Exercise Solutions (condensed)
Sources: Exercises weeks 2, 3, 4, 6 (+ official solutions), Control exercises, Vision exercises, motor exercise, reflected-inertia note. Prev: [[10 Project Assignment]] · Next: [[12 Past Exam Solutions]]

> [!tip] How to use this note
> Cover the solution, solve the problem on paper, then compare. The **method** is the point: each solution says *which rule* is applied.
> Control and vision exercises are solved in detail in [[08 Control]] §12 and [[09 Sensors and Robot Vision]] §7.

---
## Week 2: rotations and homogeneous transforms

**P1.** $30°$ about world $z$, then $60°$ about world $x$, then $90°$ about world $y$. *All world axes → pre-multiply:*
$$R=R_{y,90}R_{x,60}R_{z,30}=\begin{bmatrix}0.433&0.75&0.5\\0.25&0.433&-0.866\\-0.866&0.5&0\end{bmatrix}$$

**P2.** $\phi$ about world $x$, $\psi$ about **current** $z$, $\theta$ about world $y$. Start $R_{x,\phi}$; current $z$ → post-multiply by $R_{z,\psi}$; world $y$ → pre-multiply by $R_{y,\theta}$:
$$R=R_{y,\theta}R_{x,\phi}R_{z,\psi}=\begin{bmatrix}c\theta c\psi+s\theta s\phi s\psi&s\theta s\phi c\psi-c\theta s\psi&s\theta c\phi\\c\phi s\psi&c\phi c\psi&-s\phi\\-s\theta c\psi+c\theta s\phi s\psi&c\theta s\phi c\psi+s\theta s\psi&c\theta c\phi\end{bmatrix}$$

**P3.** Another sequence with the same $R$: rotate $\theta$ about $y$, then $\phi$ about the **current** $x$, then $\psi$ about the **current** $z$ (all current axes, same order as the product $R_yR_xR_z$).
> [!note] Rule: a product $R_1R_2R_3$ can be read **right-to-left as rotations about fixed (world) axes** ($R_3$ first) **or left-to-right as rotations about current axes** ($R_1$ first). Both describe the same final orientation.

**P4.** Rotation $\alpha$ about world $x$, translation $b$ along world $z$, rotation $\phi$ about current $y$: $T=\text{Trans}_{z,b}\,\text{Rot}_{x,\alpha}\,\text{Rot}_{y,\phi}$
$$T=\begin{bmatrix}c\phi&0&s\phi&0\\s\alpha s\phi&c\alpha&-s\alpha c\phi&0\\-c\alpha s\phi&s\alpha&c\alpha c\phi&b\\0&0&0&1\end{bmatrix}$$

**P5.** Read the frames by observation (columns = axes of the new frame in the old one, last column = origin; $a,b,c,d,e$ are the lengths in the figure).
- Origins: $o^0_1=(0,\,c+e,\,a-d)$, $o^1_2=(b,\,a-d,\,0)$, $o^2_3=(e,0,a)$, $o^3_4=(d,0,c)$, $o^4_5=(b,0,d)$, $o^0_5=(0,0,a)$.
- $A^0_1=\begin{bmatrix}-1&0&0&0\\0&0&-1&c+e\\0&-1&0&a-d\\0&0&0&1\end{bmatrix}$ ($x_1=-x_0,\ y_1=-z_0,\ z_1=-y_0$)
- $A^1_2=\begin{bmatrix}0&-1&0&b\\0&0&-1&a-d\\1&0&0&0\\0&0&0&1\end{bmatrix}$ ($x_2=z_1,\ y_2=-x_1,\ z_2=-y_1$)
- $A^2_3=\begin{bmatrix}0&0&1&e\\0&1&0&0\\-1&0&0&a\\0&0&0&1\end{bmatrix}$, $\ A^3_4=\begin{bmatrix}0&0&-1&d\\1&0&0&0\\0&-1&0&c\\0&0&0&1\end{bmatrix}$, $\ A^4_5=\begin{bmatrix}0&0&-1&b\\1&0&0&0\\0&-1&0&d\\0&0&0&1\end{bmatrix}$
- $A^0_5=\begin{bmatrix}0&0&-1&0\\1&0&0&0\\0&-1&0&a\\0&0&0&1\end{bmatrix}$ (check: $x_5=y_0,\ y_5=-z_0,\ z_5=-x_0$).

**P6.** Triangle with sides 30/40/50 cm: $\sin\beta=0.6$, $\cos\beta=0.8$.
- $A^0_1=\begin{bmatrix}1&0&0&-0.35\\0&-1&0&0.3\\0&0&-1&0.1\\0&0&0&1\end{bmatrix}$, $\ A^1_2=\begin{bmatrix}0&0&1&0\\-1&0&0&0.3\\0&-1&0&0.1\\0&0&0&1\end{bmatrix}$
- $A^2_3=\begin{bmatrix}0&0.6&0.8&0\\0&-0.8&0.6&0.5\\1&0&0&0\\0&0&0&1\end{bmatrix}$, $\ A^3_4=\begin{bmatrix}-1&0&0&0.35\\0&1&0&0\\0&0&-1&0\\0&0&0&1\end{bmatrix}$
- $A^0_4=\begin{bmatrix}-1&0&0&0\\0&0.6&-0.8&0\\0&-0.8&-0.6&0.5\\0&0&0&1\end{bmatrix}$

---
## Week 3: DH, forward & inverse kinematics

**P1 (Stanford arm, $d_2=0.1$ m).** Table: see [[03 DH Convention and Forward Kinematics]]. Forward model (position column):
$$o^0_6=\begin{bmatrix}c_1d_3s_2-0.1s_1\\ d_3s_1s_2+0.1c_1\\ c_2d_3\end{bmatrix}$$
(orientation entries $r_{ij}$ are long expressions in $c_1…c_6$; $r_{13}=c_1(c_2c_4s_5+c_5s_2)-s_1s_4s_5$, $r_{33}=c_2c_5-c_4s_2s_5$.)
**Yaw–pitch–roll** of $R^0_6$: $R=R_{z,\phi}R_{y,\theta}R_{x,\psi}$.
- If $c\theta\ne0$: $\theta=\text{atan2}(-r_{31},\pm\sqrt{r_{11}^2+r_{21}^2})$, $\psi=\text{atan2}(r_{32},r_{33})$, $\phi=\text{atan2}(r_{21},r_{11})$.
- $\theta=90°$: only $\psi-\phi=\text{atan2}(r_{22},r_{12})$. $\theta=270°$: only $\psi+\phi=\text{atan2}(r_{22},-r_{12})$; the other angle is free.

**P2 (planar 2R, links 1 m).** $T^0_2$ as in [[03 DH Convention and Forward Kinematics]].
Inverse (tip $r_{14},r_{24}$ known): law of cosines → $-\cos\theta_2=\dfrac{2-r_{14}^2-r_{24}^2}{2}$, so
$$c_2=\frac{r_{14}^2+r_{24}^2}2-1,\quad\theta_2=\text{atan2}\big(\pm\sqrt{1-c_2^2},c_2\big)\ (+:\text{elbow down},\ -:\text{elbow up}),\quad\theta_1=\text{atan2}(r_{24},r_{14})-\text{atan2}(s_2,\,1+c_2)$$

---
## Week 4: velocity kinematics

**P1.** $H^0_1$ is constant → the frames are fixed to each other → velocity is a free vector: $v^0=R^0_1v^1=\begin{bmatrix}0&-1&0\\1&0&0\\0&0&1\end{bmatrix}\begin{bmatrix}3\\1\\0\end{bmatrix}=\begin{bmatrix}-1\\3\\0\end{bmatrix}$.

**P2 (3-link planar, centre of link 2 at $a_c=a_2/2$).** $o_c=(a_1c_1+a_cc_{12},\ a_1s_1+a_cs_{12},\ 0)$, $z_0=z_1=(0,0,1)$. Joint 3 doesn't affect $o_c$, so
$$J=\begin{bmatrix}-a_1s_1-a_cs_{12}&-a_cs_{12}\\a_1c_1+a_cc_{12}&a_cc_{12}\\0&0\\0&0\\0&0\\1&1\end{bmatrix},\quad\begin{bmatrix}v\\\omega\end{bmatrix}=J\begin{bmatrix}\dot\theta_1\\\dot\theta_2\end{bmatrix}$$

**P3 (cylindrical RPP).** $o_0=0$, $o_3=(-d_3s_1,\ d_3c_1,\ d_1+d_2)$, $z_0=z_1=(0,0,1)$, $z_2=(-s_1,c_1,0)$:
$$J=\begin{bmatrix}-d_3c_1&0&-s_1\\-d_3s_1&0&c_1\\0&1&0\\0&0&0\\0&0&0\\1&0&0\end{bmatrix}$$

**P4 (PRR planar robot on a vertical slide, XZ plane).** DH: $(0,\,d_1^*,\,0,\,-90°)$ and $(\theta_2^*,\,0,\,a_2,\,0)$; frame 3 is the end-effector frame ($z_3$ approach, $y_3$ closing), defining $\theta_3$ as the angle from $x_2$ to $x_3$ about $z_2$.
$$T^0_3=\begin{bmatrix}c_{23}&0&s_{23}&a_2c_2+a_3s_{23}\\0&1&0&0\\-s_{23}&0&c_{23}&d_1-a_2s_2+a_3c_{23}\\0&0&0&1\end{bmatrix}$$
$$J=\begin{bmatrix}0&-a_2s_2+a_3c_{23}&a_3c_{23}\\0&0&0\\1&-a_2c_2-a_3s_{23}&-a_3s_{23}\\0&0&0\\0&1&1\\0&0&0\end{bmatrix}$$
(column 1 prismatic: $z_0$; columns 2,3: $z_i\times(o_3-o_i)$ with $z_1=z_2=(0,1,0)$.) *A different definition of $\theta_3$ changes the entries.*

---
## Week 6: dynamics of the 2R arm
Full derivation, $D$, $c_{ijk}$, $g$ and the equations of motion: [[07 Manipulator Dynamics]] (Example B). Key results:
$D=\dfrac{mL^2}{1200}\begin{bmatrix}2002+1200\cos\theta_2&401+600\cos\theta_2\\401+600\cos\theta_2&401\end{bmatrix}$, $\ g_1=mgL(1.5\cos q_1+0.5\cos(q_1+q_2))$, $\ g_2=0.5\,mgL\cos(q_1+q_2)$.

---
## Control exercises (07)
Summaries of A1–E2 (rotor, RLC, P/PI/PID on both plants, block reduction, DC motor, independent-joint control) with all the formulas and pole tables are in [[08 Control]] §12. Key numbers to remember: poles of B1 $=-14,-24,-44,-404$; RLC $\omega_n=44.72$, $\zeta=0.224$; E2 $=\frac{0.5s^2+3s+4.5}{1.5s^2+9s+9.5}$.

## Vision exercises (08)
Drone (7.636 m/mm, $Z=61.09$ m), couch (2.00 × 0.91 × 0.92 m, cannot pass the 0.8 m door), people (1.72 m vs 1.75 m) are in [[09 Sensors and Robot Vision]] §7. The **VisionExam** is solved in [[12 Past Exam Solutions]].

## Motor exercise (31383, DC motor in MATLAB/Simulink) 🟦
Task: derive $\Theta(s)/V(s)$ from the motor equations (armature voltage → current → torque → load inertia and friction → speed → angle) and build a Simulink model with a P, PD or PID controller. 
The provided solution diagram has the chain **`PID → Km/(La·s+Ra) → 1/(J·s+b) → 1/s → Scope`**, with the back-emf **`Kb`** fed back from the speed to the first sum (and the unity feedback from $\theta$ to the controller input). 
> [!note] The equations of the exercise are embedded as images in the PDF and were not extractable; this structure is taken from the solution block diagram and agrees with [[08 Control]] §7.

## Reflected inertia (handwritten note) 🟦
Load torque seen by the motor: $\tau_{Lm}=\frac1{n^2}J_L\ddot\theta_m+\frac1n\tau_L$. Substituting into the motor equation:
$$J_m\ddot\theta_m+D_m\dot\theta_m=\tau_m-\tfrac1{n^2}J_L\ddot\theta_m-\tfrac1n\tau_L\ \Rightarrow\ \Big(J_m+\tfrac{J_L}{n^2}\Big)\ddot\theta_m+D_m\dot\theta_m=\tau_m-\tfrac1n\tau_L$$
**$J_L/n^2$ = reflected inertia (RI).** $n\uparrow\ \Rightarrow$ RI$\downarrow$.
