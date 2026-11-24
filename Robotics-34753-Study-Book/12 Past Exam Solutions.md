---
tags: [robotics, 34753, exam, solutions]
---
# 12 Past Exam: Winter 2025-26 MCQ, fully solved
Sources: `MCQ_Printer_friendly_version.pdf` (21 pages, Q1.1–Q5.2) and `VisionExam.pdf`. Prev: [[11 Exercise Solutions]] · Next: [[13 Formula Sheet]]

> [!warning] How reliable are these answers?
> 🧮 All numbers were **recomputed by me** (re-checked 21 Nov 2026 with a Python script and by re-reading every figure in the PDF). The exam PDF has no answer key, so nothing here is "official". Confidence:
> **High:** Q1.1, Q1.2, Q2.2, Q2.3, Q3.1, Q3.2, Q4.1–Q4.4, VisionExam (each matches exactly one answer option; Q1.2/Q2.2 frame directions re-checked on the figure).
> **Medium-high:** Q2.1 (figure re-checked: the dashed-circle motor's axis is parallel to the shoulder pivot → joint 2; the arrow points at the red forearm tube → link 4), Q5.2 (both plausible readings give ≈0.01 rad, see note).
> **Medium:** Q5.1 (3 vs 2 poles is a wording question).
> Try each question yourself first. 🎯

---
# Section 1: Position and orientation kinematics

## Q1.1: Build $T$ from a sequence of moves
Sequence: (1) rot 60° about **current** $x$ → (2) trans 50 mm along **current** $y$ → (3) rot 40° about **global** $z$ → (4) trans 30 mm along **global** $x$.

**Method** ([[02 Rotations and Homogeneous Transformations]]): current → post-multiply, global → pre-multiply.
$$T=\text{Trans}_{x,30}\;\text{Rot}_{z,40°}\;\text{Rot}_{x,60°}\;\text{Trans}_{y,50}$$
$$T=\begin{bmatrix}0.766&-0.321&0.557&13.93\\0.643&0.383&-0.663&19.15\\0&0.866&0.5&43.30\\0&0&0&1\end{bmatrix}$$
**Answer (pick the vector):**
- Column 1 = $(0.766,\ 0.643,\ 0)$
- Column 2 = $(-0.321,\ 0.383,\ 0.866)$
- Column 3 = $(0.557,\ -0.663,\ 0.5)$
- Column 4 = $(13.93,\ 19.15,\ 43.3)$

*Check:* origin $=\text{Trans}_x\text{Rot}_z\,[0,\ 50\cos60°,\ 50\sin60°]^T=\text{Rot}_{z,40}[0,25,43.3]^T+[30,0,0]^T=[-16.07+30,\ 19.15,\ 43.3]$ ✔. (Distractors with 22.98, 59.64… come from wrong multiplication order.)

## Q1.2: Frame $\{1\}$ on the box w.r.t. camera frame $\{0\}$
Figure: room $L\times H\times W$ (L = left-right, H = height, W = depth). Camera frame $\{0\}$ at the **top corner where the front wall meets the right wall**: $x_0$ points **left** (along $L$), $y_0$ points **into the room** (depth), $z_0$ points **down**. The box ($\ell\times h\times w$) stands on the floor against the left wall, a distance $a$ from the front wall. Frame $\{1\}$ is at the **top-front-right corner of the box**: $x_1$ points into the room, $y_1$ points left, $z_1$ points up.

**Method:** columns = axes of $\{1\}$ in $\{0\}$:
- $x_1$ (into the room) $=+y_0$ → **$(0,1,0,0)$**
- $y_1$ (left) $=+x_0$ → **$(1,0,0,0)$**
- $z_1$ (up) $=-z_0$ → **$(0,0,-1,0)$**
- Origin: distance from the right wall along $x_0$ = $L-\ell$ (box spans $\ell$ from the left wall); along $y_0$ = $a$; along $z_0$ (down from the ceiling) = $H-h$ → **$(L-\ell,\ a,\ H-h,\ 1)$**

**Answer:** Col 1 = $(0,1,0,0)$, Col 2 = $(1,0,0,0)$, Col 3 = $(0,0,-1,0)$, Col 4 = $(L-\ell,\,a,\,H-h,\,1)$.
*Check handedness:* $x_1\times y_1=(0,1,0)\times(1,0,0)=(0,0,-1)=z_1$ ✔. The distractor $(L-\ell,\,W-w,\,H-h)$ would put the box against the **back** wall.

---
# Section 2: DH frames and inverse kinematics

## Q2.1: Counting joints/links on an exploded 6-axis robot 🧮 (read from the figure)
| Question | Answer |
|---|---|
| Which joint is driven by the motor in the dashed circle? | **2** (motor mounted on the turret, horizontal axis → shoulder joint; joint $i$ is driven by a motor on link $i-1$) |
| How many joints? | **6** |
| How many revolute? | **6** |
| How many prismatic? | **0** |
| Number of the link the thick arrow points to? | **4** (dark forked wrist element: base 0, turret 1, upper arm 2, elbow/forearm housing 3, fork 4, wrist cube 5, flange 6) |
> [!warning] Check the picture again in the real exam: count the red/dark bodies from the base to the flange. The link answer depends on which body the arrow touches.

## Q2.2: DH parameters from a figure (UR-style arm, 6 DoF)
Figure: frames $\{0\}\dots\{5\}$ given by $x$ and $z$ axes. Dimensions: base to shoulder 90 mm; two 400 mm links; 180 mm and 90 mm wrist offsets.
Let $X$ = along the arm (away from the base), $Y$ = horizontal sideways (direction of $z_1$), $Z$ = up. From the figure:

| frame | $z_i$ | $x_i$ |
|---|---|---|
| 0 | $+Z$ | $-X$ (points away from the arm) |
| 1 | $+Y$ | $+X$ |
| 2 | $+Y$ | $-X$ (points back toward the base) |
| 3 | $+Y$ | $+X$ |
| 4 | $+Z$ | $+X$ |
| 5 | $+Y$ | $+X$ |

**Reading each parameter** (angle $x_{i-1}\to x_i$ about $z_{i-1}$; $d$ along $z_{i-1}$; $a$ along $x_i$; $\alpha$ from $z_{i-1}\to z_i$ about $x_i$):

| $i$ | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ | reasoning |
|---|---|---|---|---|---|
| 1 | **180** | **90** | **0** | **−90** | $x_0=-X\to x_1=+X$; $o_1$ is 90 mm above $o_0$; $z_0=Z\to z_1=Y$ is a $-90°$ turn about $x_1=X$ |
| 2 | **180** | **0** | **−400** | **0** | $x_1=X\to x_2=-X$; $o_2$ is $+400X=-400\,x_2$ from $o_1$; $z_1\parallel z_2$ |
| 3 | **180** | **0** | **400** | **0** | $x_2=-X\to x_3=+X$; $o_3$ is $+400X=+400\,x_3$ from $o_2$ |
| 4 | **0** | **180** | **0** | **90** | $x_3=x_4$; $o_4$ is 180 mm along $z_3=+Y$; $z_3=Y\to z_4=Z$ is $+90°$ about $x_4=X$ |
| 5 | **0** | **−90** | **0** | **−90** | $x_4=x_5$; $o_5$ is 90 mm **below** $o_4$ along $z_4=Z$ → $d_5=-90$; $z_4=Z\to z_5=Y$ is $-90°$ about $X$ |

**Answer grid:** $\theta_1,\theta_2,\theta_3=180$; $\theta_4,\theta_5=0$ · $d_1=90$, $d_2=d_3=0$, $d_4=180$, $d_5=-90$ · $a_1=0$, $a_2=-400$, $a_3=400$, $a_4=a_5=0$ · $\alpha_1=-90$, $\alpha_2=\alpha_3=0$, $\alpha_4=90$, $\alpha_5=-90$.
*Sanity checks:* (i) $o_2=o_1+A$: with $\theta_2=180°$ and $a_2=-400$ the origin moves $a_2\cdot x_2=(-400)(-X)=+400X$ ✔. (ii) Every option value ($\pm400,\pm90,0,180$) appears in the grid and all of them are used. (iii) This is the usual UR DH pattern (negative $a_2$, positive wrist offsets).
> [!tip] 🎯 Do it in this order: draw $X,Y,Z$ → write $z_i$ and $x_i$ in those axes for every frame → then each parameter is one subtraction or one angle.

## Q2.3: Inverse kinematics of a 2R arm
Given $T^0_2$ with $a_1=391$, $a_2=302$ (mm) and target $o_2=(0,\,592.7)$, elbow down.
$$\cos\theta_2=\frac{592.7^2-391^2-302^2}{2\cdot391\cdot302}=0.4540\Rightarrow\theta_2=63°$$
$$\theta_1=\text{atan2}(592.7,0)-\text{atan2}(302\sin63°,\ 391+302\cos63°)=90°-27°=63°$$
**Answer: $\theta_1=63°,\ \theta_2=63°$.** *Check:* $x=391\cos63°+302\cos126°=0$ ✔, $y=391\sin63°+302\sin126°=592.7$ ✔. (The elbow-up solution is $(117°,-63°)$, not among the options.) See [[04 Inverse Kinematics]].

---
# Section 3: Velocity kinematics and trajectory planning

## Q3.1: Tip velocity from a given Jacobian
$J_v=\begin{bmatrix}-0.597&-0.244\\0&-0.178\end{bmatrix}$ m/rad → $v_A=J_v\dot\theta$.

| $\dot\theta_1,\dot\theta_2$ [rad/s] | $v_A$ [m/s] |
|---|---|
| (1, 1) | $(-0.597-0.244,\ -0.178)=\mathbf{(-0.841,\ -0.178)}$ |
| (1, −1) | $(-0.597+0.244,\ +0.178)=\mathbf{(-0.353,\ 0.178)}$ |
| (1.371, −3.354) | $(-0.818+0.818,\ 0.597)=\mathbf{(0,\ 0.597)}$ |
| (−0.409, 0) | $(0.244,\ 0)=\mathbf{(0.244,\ 0)}$ |

See [[05 Velocity Kinematics and Jacobian]]. Row 3 is the joint velocity that gives a **pure $y$-velocity** of 0.597 m/s: $J_v$ is square and invertible here ($\det=0.597\cdot0.178\ne0$), so $\dot\theta=J_v^{-1}(0,0.597)^T$ is unique. 🎯 If an option's $x$-component doesn't vanish, you've mixed up the columns.

## Q3.2: Quintic coefficients
$t_A=0$: $q=\pi/4,\ \dot q=0,\ \ddot q=0$; $t_B=2$: $q=0,\ \dot q=1,\ \ddot q=0$.
$c_0=\pi/4=\mathbf{0.7854}$, $c_1=\mathbf0$, $c_2=\mathbf0$ (from the initial conditions), then
$$\begin{bmatrix}8&16&32\\12&32&80\\12&48&160\end{bmatrix}\begin{bmatrix}c_3\\c_4\\c_5\end{bmatrix}=\begin{bmatrix}-\pi/4\\1\\0\end{bmatrix}\Rightarrow c_3=\mathbf{-1.982},\ c_4=\mathbf{1.611},\ c_5=\mathbf{-0.3348}$$
*Check:* $q(2)=0.7854-1.982\cdot8+1.611\cdot16-0.3348\cdot32=0.7854-15.856+25.776-10.714=-0.009\approx0$ ✔ (rounding). See [[06 Trajectory Planning]].

---
# Section 4: Robot vision
Camera set-up for Q4.1–4.4: camera on the end-effector, pointing down; $T^b_c=\begin{bmatrix}1&0&0&0\\0&-1&0&0.3\\0&0&-1&0.5\\0&0&0&1\end{bmatrix}$, i.e. camera at $(X,Y,Z)=(0,\,0.3,\,0.5)$ m, $x_c=X$, $y_c=-Y$, $z_c=-Z$ (table at $Z=0$, **depth $z=500$ mm**). $\lambda=8$ mm, $\alpha_x=79.2$, $\alpha_y=120.5$ pix/mm, $o_r=250$, $o_c=160$.
Pixels: $r=o_r-\alpha_xu$, $c=o_c-\alpha_yv$ with $u=\lambda x/z$, $v=\lambda y/z$ (see the sign warning in [[09 Sensors and Robot Vision]]).
**Keyboard geometry** (read from the figure): 5 columns × 3 rows of 1 cm keys; keyboard frame $x$ right, $y$ **down**, origin at the **bottom edge, between the 3rd and 4th column**, located under the camera at $(X,Y)=(0,\,0.3)$ m. Top row C D E | A F, middle H I L Q P, bottom K X Z S O. Corners: $P_1(-3,-3)$, $P_2(-3,0)$, $P_3(2,0)$, $P_4(2,-3)$ cm; key centres (cm): C $(-2.5,-2.5)$, I $(-1.5,-1.5)$, A $(0.5,-2.5)$, O $(1.5,-0.5)$, X $(-1.5,-0.5)$, Q $(0.5,-1.5)$.

## Q4.1: Image-plane positions of the corners
Corners in the camera frame (mm): $P_1(-30,-30)$, $P_2(-30,0)$, $P_3(20,0)$, $P_4(20,-30)$ ($x_c=x_{key}$, $y_c=y_{key}$ since both point right/down).
$u=8\cdot x/500$: $-0.48,\ -0.48,\ 0.32,\ 0.32$; $v$: $-0.48,\ 0,\ 0,\ -0.48$.
**Answer:** $P_c=\begin{bmatrix}-0.48&-0.48&0.32&0.32\\-0.48&0&0&-0.48\end{bmatrix}$ mm (first option).

## Q4.2: Pixel sequence for "CIAO" (key centres, depth 500 mm)
| key | $(x_c,y_c)$ mm | $(u,v)$ mm | $r=250-79.2u$ | $c=160-120.5v$ |
|---|---|---|---|---|
| C | (−25, −25) | (−0.40, −0.40) | 281.7 → **282** | 208.2 → **208** |
| I | (−15, −15) | (−0.24, −0.24) | 269.0 → **269** | 188.9 → **189** |
| A | (5, −25) | (0.08, −0.40) | 243.7 → **244** | 208.2 → **208** |
| O | (15, −5) | (0.24, −0.08) | 231.0 → **231** | 169.6 → **170** |

**Answer:** $C=[282;\,208]$, $I=[269;\,189]$, $A=[244;\,208]$, $O=[231;\,170]$ (third non-"impossible" option).

## Q4.3: Positions in the global (base) frame
$p^b=T^b_cp^c$: $X=x_c$, $Y=0.3\,\text{m}-y_c$, $Z=0.5-z_c=0$ (table).
$C=(-25,\ 325,\ 0)$, $I=(-15,\ 315,\ 0)$, $A=(5,\ 325,\ 0)$, $O=(15,\ 305,\ 0)$ **mm** (the option with these four vectors).

## Q4.4: Camera moved to $T^b_{c2}$ (camera at $X=-0.03$, $Y=0.33$ m, same height)
Keys in the base frame: $X$-key $(X,Y)=(-15,\,305)$ mm, $Q$-key $(5,\,315)$ mm. In the new camera frame: $x_c=X+30$, $y_c=330-Y$, $z=500$.
- $X$: $(x_c,y_c)=(15,25)$ → $u=0.24,\ v=0.40$ → $r=250-19.0=\mathbf{231}$, $c=160-48.2=\mathbf{112}$
- $Q$: $(35,15)$ → $u=0.56,\ v=0.24$ → $r=250-44.4=\mathbf{206}$, $c=160-28.9=\mathbf{131}$

**Answer:** $X=[231;\,112]$, $Q=[206;\,131]$.

---
# Section 5: Motor control

## Q5.1: Number of poles
Parameters: $L_m=0.001$ H, $R_m=0.9\,\Omega$, $K_m=0.12$ V·s/rad (back-emf), $K_i=0.12$ N·m/A, $J_m=0.08$, $D_m=0.027$, $J_l=0.18$, $n=20$. With **non-negligible electrical dynamics** the voltage-to-angle model is
$$\frac{\Theta_m}{V}=\frac{K_i}{s\big[(L_ms+R_m)(J_{eq}s+D_{eq})+K_mK_i\big]},\quad J_{eq}=J_m+\tfrac{J_l}{n^2}$$
Denominator: $s\times$(quadratic) → **3 poles** (one at the origin, plus the electrical and mechanical ones).
**Answer: 3.** (⚠️ If the question meant the speed output $\omega_m$ only, it would be 2; with negligible $L$ the angle model has 2 poles. The question stresses "non-negligible electrical dynamics", i.e. the 3rd-order model from the lecture.) See [[08 Control]] §7.

## Q5.2: Steady-state error, P controller with constant load torque
$V=k_p(\theta^\star_m-\theta_m)$, $k_p=3.78$, $\theta^\star_m=1.5$ rad, $\tau_l=0.05$ N·m.
In steady state $\dot\theta_m=0$ and $i=\text{const}$ → no back-emf and no inductor voltage: $V=R_mi$; the motor torque balances the load torque reflected through the gears: $K_ii=\tau_l/n$.
$$e_{ss}=\frac{V}{k_p}=\frac{R_m\,\tau_l}{n\,K_i\,k_p}=\frac{0.9\cdot0.05}{20\cdot0.12\cdot3.78}=0.0050\ \text{rad}$$
**Answer: the option "approximately 0.01 rad"** (the only option of that order of magnitude; not 0 because there is no integrator, not infinite because the loop is stable). 
> [!warning] Two readings, same option
> - **$k_p$ outputs a voltage** (book model, above): $e_{ss}=0.005$ rad.
> - **$k_p$ outputs a torque** directly against $\tau_l$ (no motor/gear chain): $e_{ss}=\tau_l/k_p=0.05/3.78=0.013$ rad.
>
> Both round to **"approximately 0.01 rad"**. The distractor **−0.0128** is $-\tau_l/(k_p+K_m)$: wrong sign, and it adds quantities with different units. **0** would need an integrator; **∞** would need an unstable loop; **0.075 m** has the wrong unit. 🎯 Eliminate by sign, unit and order of magnitude first, then compute.

---
# VisionExam (separate paper) 🧮
**Q4.1:** points in the camera frame, $\lambda=8$ mm, $\alpha_x=79.2$, $\alpha_y=120.5$, $o_r=o_c=250$. $r=250-79.2\cdot8x/z$, $c=250-120.5\cdot8y/z$:
- $P_1(-0.1,-0.1,1)$: $r=250+63.4=313$, $c=250+96.4=346$ → $[313;346]$; $P_2$: $[123;346]$; $P_3$: $[123;57]$; $P_4$: $[313;57]$
- $z=2$ doubles the denominator: $P_5=[282;298]$, $P_6=[187;298]$, $P_7=[187;154]$, $P_8=[282;154]$
**Answer: the second option** ($P_5\dots P_8$ = 298/154 rows). The first option has the wrong rows (282/187), the third forgets that depth halves the offsets.

**Q4.2:** Points at $r_1=313$, $r_2=155$ (same $c$), 0.3 m apart on a plane parallel to the sensor. $\Delta r=158$ pix → $158/79.2=1.995$ mm.
$$d=z=\lambda\frac{\Delta X}{\Delta u}=8\cdot\frac{0.3\ \text{m}}{1.995}=1.20\ \text{m}$$
**Answer: $d=1.2$ m.**

---
## What to practise next
1. Re-solve Q1.1 and Q3.2 by hand, and Q2.3 with a calculator, **in under 5 minutes each**.
2. Draw the DH frames for a *different* robot (e.g. the Stanford arm, SCARA) and read the table.
3. Make your own Q4-type problem: move the camera and recompute pixels of a key.
4. Explain without notes: why 3 poles? why does $e_{ss}$ shrink with the gear ratio?
