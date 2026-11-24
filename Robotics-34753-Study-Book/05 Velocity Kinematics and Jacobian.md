---
tags: [robotics, 34753, kinematics, jacobian]
---
# 05 Velocity Kinematics and the Jacobian
Sources: Lecture 4, Lecture 6 (repetition), Exercises week 4, Exam Q3.1. Prev: [[04 Inverse Kinematics]] · Next: [[06 Trajectory Planning]]

## Angular velocity as a vector 🟦
- Rotation about unit axis $k$ with angle $\theta$; angular velocity $\omega=\dot\theta\,k$.
- Linear velocity of a point at position $r$ from the axis: $v=\omega\times r$.
- Cross product as a matrix with the **skew-symmetric matrix**
$$S(\omega)=\begin{bmatrix}0&-\omega_z&\omega_y\\\omega_z&0&-\omega_x\\-\omega_y&\omega_x&0\end{bmatrix},\qquad v=\omega\times r=S(\omega)\,r$$

**Properties of $S$:** $S+S^T=0$ · $S(\alpha a+\beta b)=\alpha S(a)+\beta S(b)$ · $S(Ra)=R\,S(a)\,R^T$ for orthonormal $R$ · $x^TS(a)x=0$ for any $x$.

> [!example] Lecture exercise: recover $\omega$, $\dot\theta$, $k$ from $S=\begin{bmatrix}0&3&-1\\-3&0&-4\\1&4&0\end{bmatrix}$ 🧮
> Match entries: $-\omega_z=3\Rightarrow\omega_z=-3$; $\omega_y=-1$; $-\omega_x=-4\Rightarrow\omega_x=4$. So $\omega=(4,-1,-3)^T$, $\dot\theta=\|\omega\|=\sqrt{26}\approx5.10$, $k=\omega/\sqrt{26}$.

## Derivative of a rotation matrix 🟦
From $RR^T=I$: $\dot R R^T+R\dot R^T=0$ → $\dot RR^T$ is skew-symmetric.
$$\dot R(t)=S(\omega(t))\,R(t),\qquad \text{for a single axis: }\ \frac{dR}{d\theta}=S(k)R(\theta)$$
Example: $R_{z,\theta}$ → $\frac{dR_z}{d\theta}R_z^T=\begin{bmatrix}0&-1&0\\1&0&0\\0&0&0\end{bmatrix}=S(z)$.

## Addition of angular velocities 🟦
Let $\omega^{i-1}_{i-1,i}$ = angular velocity of link $i$ w.r.t. link $i-1$, expressed in frame $i-1$. Then the end-effector's angular velocity in the world frame is
$$\omega^0_{0,n}=\omega^0_{0,1}+R^0_1\omega^1_{1,2}+\dots+R^0_{n-1}\omega^{n-1}_{n-1,n}=\omega^0_{0,1}+\omega^0_{1,2}+\dots+\omega^0_{n-1,n}$$
For a **revolute** joint $i$ the contribution is $\dot\theta_i\,z_{i-1}$.

## Linear velocity of a point fixed on link $i$ 🟦
$p^0=R^0_ip^i+o^0_i$, with $\dot p^i=0$:
$$\dot p^0=S(\omega^0_i)R^0_ip^i+\dot o^0_i=\omega\times r+v,\qquad r=p^0-o^0_i$$
A **free vector** such as a velocity changes frame with $R$ only: $v^0=R^0_1v^1$ (week 4, problem 1: $v^0=R^0_1[3,1,0]^T=[-1,3,0]^T$).

## The manipulator Jacobian 🟦
Maps joint velocities to the end-effector velocity $\xi=[v^0_n;\ \omega^0_n]$ (6×1):
$$\xi=\begin{bmatrix}v_n^0\\\omega_n^0\end{bmatrix}=J(q)\,\dot q=\begin{bmatrix}J_v\\J_\omega\end{bmatrix}\dot q,\qquad \text{with } S(\omega^0_n)=\dot R^0_nR^{0\,T}_n$$
Column $i$ of $J=[J_1\ \cdots\ J_n]$:
$$J_i=\begin{cases}\begin{bmatrix}z_{i-1}\\0\end{bmatrix}&\text{prismatic}\\[3mm]\begin{bmatrix}z_{i-1}\times(o_n-o_{i-1})\\z_{i-1}\end{bmatrix}&\text{revolute}\end{cases}$$
**Where $z_{i-1}$ and $o_{i-1}$ come from:** $T^0_{i-1}=\begin{bmatrix}r_{11}&r_{12}&r_{13}&o_x\\r_{21}&r_{22}&r_{23}&o_y\\r_{31}&r_{32}&r_{33}&o_z\\0&0&0&1\end{bmatrix}$ → $z_{i-1}=(r_{13},r_{23},r_{33})^T$ (third column), $o_{i-1}=(o_x,o_y,o_z)^T$ (fourth column). $z_0=[0,0,1]^T$, $o_0=0$.

> [!tip] 🎯 Procedure
> 1. Get all $T^0_i$ (forward kinematics).  2. Extract $z_0\dots z_{n-1}$, $o_0\dots o_n$.  3. Per column: revolute ⇒ $z\times(o_n-o_{i-1})$ over $z$; prismatic ⇒ $z$ over $0$.
> The Jacobian **for a point other than the tip** (e.g. a link's centre of mass) uses that point instead of $o_n$ and ignores joints beyond that link (columns = 0).

### Example: planar 2R 🟦
$o_0=0$, $o_1=(a_1c_1,a_1s_1,0)$, $o_2=(a_1c_1+a_2c_{12},\,a_1s_1+a_2s_{12},\,0)$, $z_0=z_1=(0,0,1)$:
$$J=\begin{bmatrix}-a_1s_1-a_2s_{12}&-a_2s_{12}\\a_1c_1+a_2c_{12}&a_2c_{12}\\0&0\\0&0\\0&0\\1&1\end{bmatrix},\qquad \det J_v=a_1a_2\sin\theta_2$$

### Example: cylindrical (RPP), week 4 problem 3 🟦
$$J=\begin{bmatrix}-d_3c_1&0&-s_1\\-d_3s_1&0&c_1\\0&1&0\\0&0&0\\0&0&0\\1&0&0\end{bmatrix}$$
(joint 1 revolute: $z_0\times o_3$; joints 2 and 3 prismatic: $z_1=[0,0,1]^T$, $z_2=[-s_1,c_1,0]^T$.)

### Exam Q3.1 style: apply a given $J$ 🧮
Given $J_v=\begin{bmatrix}-0.597&-0.244\\0&-0.178\end{bmatrix}$ m/rad: $v_A=J_v\dot\theta$, e.g. $\dot\theta=(1,1)\Rightarrow v_A=(-0.841,-0.178)$. Full table in [[12 Past Exam Solutions]].
Reading the geometry from $J$: column 1 of a planar RR arm is $(-y_A,\,x_A)$ → tip at $(0,\,0.597)$ m; column 2 is $(-(y_A-y_1),\,x_A-x_1)$ → elbow at $(0.178,\,0.353)$ m.

## Singularities 🟦
- A configuration $q^s$ is **singular** if $\text{rank}\,J(q^s)<\max_q\text{rank}\,J(q)$. For a square $J$: $\det J=0$.
- **Implications:** some directions of motion unreachable; **infinite joint velocity** for bounded tip velocity; bounded joint torques ⇔ **unbounded tip forces**/torques; often at workspace boundaries; tiny changes of link geometry can make positions unreachable.
- 2R arm: singular at $\theta_2=0$ (stretched) or $\pi$ (folded). 📖 For a 6-DoF arm with spherical wrist $\det J$ factorises into an *arm* part and a *wrist* part (wrist singular when $z_3,z_5$ align).

## Static force/torque 🟦
$$\tau=J^T(q)\,F$$
$F$ = 6-vector (force, moment) applied at the end-effector, $\tau$ = joint torques/forces needed to hold it. **Work-conjugate pairs:** $(\xi,F)$ and $(\dot q,\tau)$, so power $=\xi^TF=\dot q^T\tau$. (Used in project part 3: motor loads.)

## Inverse velocity 🟦
$$\dot q=J^{+}\xi,\qquad J^+=\begin{cases}J^T(JJ^T)^{-1}&m<n\ \text{(redundant arm)}\\J^{-1}&m=n\\(J^TJ)^{-1}J^T&m>n\end{cases}$$
In general with the **null-space** term (self-motion that doesn't move the tip):
$$\dot q=J^+\xi+(I_{n\times n}-J^+J)\,b$$
Lecture example (ball catcher, 4-joint arm): the pseudo-inverse of the Jacobian at the catching pose gives $\dot q(t_B)=(3.596,\,9.246,\,-7.746,\,-5.096)^T$, and the slide then checks that $J\dot q$ reproduces the desired tip velocity $\xi$ (always do this check: a pseudo-inverse always returns *something*, even if $\xi$ is not achievable). This $\dot q(t_B)$ is then used as the end boundary condition for [[06 Trajectory Planning]].

## Self-check
- What does a zero column of $J_\omega$ tell you? → a prismatic joint (no rotation contribution).
- Why can't you invert $J$ at a singularity? → $\det J=0$, rank drops.
- In which frame is $\xi=J\dot q$ expressed? → the frame in which $z_{i-1},o_i$ were expressed (normally the base frame $\{0\}$); change frame with $\text{diag}(R,R)$. 📖
