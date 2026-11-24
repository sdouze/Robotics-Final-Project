---
tags: [robotics, 34753, kinematics, inverse-kinematics]
---
# 04 Inverse Kinematics
Sources: Lecture 3 (second half), Lecture 4 (repetition), Exercises week 3, Exam Q2.3. Prev: [[03 DH Convention and Forward Kinematics]] · Next: [[05 Velocity Kinematics and Jacobian]]

## The problem 🟦
- **Forward:** known $q_1\dots q_n$ → find $H = A_1(q_1)\cdots A_n(q_n)$.
- **Inverse:** known $H$ (position **and** orientation) → solve $A_1(q_1)\cdots A_n(q_n)=H$ for $q_1\dots q_n$.
- 12 nonlinear equations (3 position + 9 rotation entries, only 6 independent) → generally no closed form.
- **Solutions are not unique:** redundant manipulators (infinitely many), elbow-up/elbow-down, shoulder left/right, wrist flip. 📖 A 6R arm with spherical wrist has up to **8** solutions.
- Solutions may not exist (target outside workspace) and some configurations are singular.

## Kinematic decoupling (spherical wrist) 🟦
If the last three joint axes $z_3, z_4, z_5$ intersect in one point $o_c$ (the **wrist centre**), their rotations do **not** move $o_c$. So split the problem:

1. **Inverse position:** find $q_1,q_2,q_3$ that place the wrist centre
$$o_c^0=o^0_6-d_6\,R^0_6\begin{bmatrix}0\\0\\1\end{bmatrix}=\begin{bmatrix}o_x-d_6r_{13}\\o_y-d_6r_{23}\\o_z-d_6r_{33}\end{bmatrix}$$
2. Compute $R^0_3(q_1,q_2,q_3)$ by forward kinematics.
3. **Inverse orientation:** find $q_4,q_5,q_6$ from
$$R^3_6(q_4,q_5,q_6)=(R^0_3)^T R^0_6$$
Given $R^0_6$ (desired) and $R^0_3$ (now known).

## Inverse position: articulated arm (RRR) 🟦
Geometric method: project the arm onto a plane and use trigonometry. With wrist centre $(x_c,y_c,z_c)$:

$$\theta_1=\text{atan2}(y_c,x_c)\quad(\text{singular if } x_c=y_c=0;\ \text{a second solution is }\theta_1+\pi)$$

Define $r^2=x_c^2+y_c^2$, $s=z_c-d_1$. **Law of cosines:**
$$\cos\theta_3=\frac{r^2+s^2-a_2^2-a_3^2}{2a_2a_3}\;,\qquad \theta_3=\text{atan2}\big(\sin\theta_3,\cos\theta_3\big),\ \ \sin\theta_3=\pm\sqrt{1-\cos^2\theta_3}$$
Two signs → **elbow-up / elbow-down**.
$$\theta_2=\text{atan2}(s,\,r)-\text{atan2}\big(a_3\sin\theta_3,\;a_2+a_3\cos\theta_3\big)$$
> [!warning] ⚠️ The slides write `Atan2(x, y)` (x first). In MATLAB/Python the above is `atan2(y, x)` order.
> (Slides: $\theta_1=\text{Atan2}(x_c,y_c)$ ⇒ MATLAB `atan2(yc, xc)`.)

If $\cos\theta_3$ comes out outside $[-1,1]$ → the point is **unreachable**.

## Inverse orientation 🟦
With a ZYZ-type wrist $R^3_6=\begin{bmatrix}c_4c_5c_6-s_4s_6&-c_4c_5s_6-s_4c_6&c_4s_5\\ s_4c_5c_6+c_4s_6&-s_4c_5s_6+c_4c_6&s_4s_5\\-s_5c_6&s_5s_6&c_5\end{bmatrix}$ this is exactly the **Euler-angle problem** of [[02 Rotations and Homogeneous Transformations]]:
$$\theta_5=\theta=\text{atan2}\big(\pm\sqrt{1-r_{33}^2},r_{33}\big),\quad\theta_4=\phi=\text{atan2}(r_{23},r_{13}),\quad\theta_6=\psi=\text{atan2}(r_{32},-r_{31})$$
(for the $+$ sign; flip signs of the arguments for the $-$ branch). Wrist singularity at $\theta_5=0$ or $\pi$.

## Planar 2R arm (exam Q2.3, week 3 problem 2) 🟦🧮
Forward model, link lengths $a_1,a_2$:
$$x=a_1c_1+a_2c_{12},\qquad y=a_1s_1+a_2s_{12}$$
**Inverse** (position of the tip $(x,y)$ known):
$$c_2=\frac{x^2+y^2-a_1^2-a_2^2}{2a_1a_2},\quad \theta_2=\text{atan2}\big(\pm\sqrt{1-c_2^2},\,c_2\big),$$
$$\theta_1=\text{atan2}(y,x)-\text{atan2}\big(a_2s_2,\;a_1+a_2c_2\big)$$
- The week-3 solution calls $\theta_2>0$ **elbow down** (with its frame convention) and $\theta_2<0$ **elbow up**. **Decide from the picture, not from the sign rule**: in Q2.3 the second link turns left (counter-clockwise) relative to the first and the figure says "elbow down", so $\theta_2>0$.
- Derivation via the triangle: the angle at the elbow is $\pi-\theta_2$, so $\cos(\pi-\theta_2)=\dfrac{a_1^2+a_2^2-r^2}{2a_1a_2}\Rightarrow -\cos\theta_2=\dots$ (same result).

### Worked numbers: Q2.3 🧮
$a_1=391$, $a_2=302$, target $(0,\,592.7)$:
- $c_2=\dfrac{592.7^2-391^2-302^2}{2\cdot391\cdot302}=0.4540\Rightarrow\theta_2=63°$
- $\theta_1=90°-\text{atan2}(302\sin63°,\,391+302\cos63°)=90°-27°=\mathbf{63°}$
- **Answer: $\theta_1=63°,\ \theta_2=63°$.** (Elbow-up would be $117°,\,-63°$.)
- Check by FK: $x=391\cos63°+302\cos126°=177.5-177.5=0$ ✔, $y=348.4+244.3=592.7$ ✔

## Strategy checklist 🎯
1. Is there a spherical wrist? → decouple.
2. Draw the arm in the plane of $(r, s)$ and mark the triangle. Law of cosines gives $\theta_3$ (or $\theta_2$ for planar).
3. Pick the solution branch (elbow up/down) **from the figure**.
4. **Always verify** by plugging the angles back into forward kinematics.
5. Watch the `atan2` argument order and degrees/radians.

## Self-check
- Why is $\theta_1=\text{atan2}(y_c,x_c)$ singular at $x_c=y_c=0$? → wrist centre on the $z_0$ axis, $\theta_1$ undetermined (any value works).
- How many unknowns does the position subproblem have for a 6R arm with spherical wrist? → 3 ($q_1,q_2,q_3$), with 3 equations.
