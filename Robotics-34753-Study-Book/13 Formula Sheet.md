---
tags: [robotics, 34753, formulas, cheatsheet]
---
# 13 Formula Sheet
All formulas on one page. Details and derivations in the chapter notes. Prev: [[12 Past Exam Solutions]] · Next: [[14 Flashcards]] · Home: [[DTU/Semester 1/Robotics/Robotics-34753-Study-Book/00 Index]]

## Rotations & transforms ([[02 Rotations and Homogeneous Transformations]])
- $R^{-1}=R^T$, $\det R=1$; $p^0=R^0_1p^1+o^0_1$.
- **Current axis → post-multiply. World axis → pre-multiply.**
- $H^{-1}=\begin{bmatrix}R^T&-R^To\\0&1\end{bmatrix}$; columns of $H$ = $x,y,z$ of the new frame, then origin.
- $R_{x}=\begin{bmatrix}1&0&0\\0&c&-s\\0&s&c\end{bmatrix}$, $R_y=\begin{bmatrix}c&0&s\\0&1&0\\-s&0&c\end{bmatrix}$, $R_z=\begin{bmatrix}c&-s&0\\s&c&0\\0&0&1\end{bmatrix}$
- **ZYZ** ($\theta\in(0,\pi)$): $\theta=\text{atan2}(\sqrt{1-r_{33}^2},r_{33})$, $\phi=\text{atan2}(r_{23},r_{13})$, $\psi=\text{atan2}(r_{32},-r_{31})$. Singular at $\sin\theta=0$.
- **RPY** $R=R_{z,\phi}R_{y,\theta}R_{x,\psi}$: $\theta=\text{atan2}(-r_{31},\sqrt{r_{11}^2+r_{21}^2})$, $\phi=\text{atan2}(r_{21},r_{11})$, $\psi=\text{atan2}(r_{32},r_{33})$. Singular at $\theta=\pm90°$.
- ⚠️ Slides use `Atan2(x,y)`; code uses `atan2(y,x)`.

## DH & FK ([[03 DH Convention and Forward Kinematics]])
$$A_i=\text{Rot}_{z,\theta_i}\text{Trans}_{z,d_i}\text{Trans}_{x,a_i}\text{Rot}_{x,\alpha_i}=\begin{bmatrix}c\theta&-s\theta c\alpha&s\theta s\alpha&ac\theta\\s\theta&c\theta c\alpha&-c\theta s\alpha&as\theta\\0&s\alpha&c\alpha&d\\0&0&0&1\end{bmatrix},\quad T^0_n=A_1\cdots A_n$$
- $\theta_i$: $x_{i-1}\to x_i$ about $z_{i-1}$ · $d_i$: along $z_{i-1}$ · $a_i$: along $x_i$ · $\alpha_i$: $z_{i-1}\to z_i$ about $x_i$.
- DH1: $x_i\perp z_{i-1}$; DH2: $x_i$ intersects $z_{i-1}$; $z_i$ = axis of joint $i+1$; $x_i\parallel z_i\times z_{i-1}$.
- Revolute: variable $\theta$. Prismatic: variable $d$. $n$ joints → $n+1$ links.

## Inverse kinematics ([[04 Inverse Kinematics]])
- Wrist centre $o_c=o_6-d_6R^0_6[0,0,1]^T$; orientation from $R^3_6=(R^0_3)^TR^0_6$.
- Articulated: $\theta_1=\text{atan2}(y_c,x_c)$; $\cos\theta_3=\dfrac{r^2+s^2-a_2^2-a_3^2}{2a_2a_3}$; $\theta_2=\text{atan2}(s,r)-\text{atan2}(a_3s_3,a_2+a_3c_3)$.
- Planar 2R: $c_2=\dfrac{x^2+y^2-a_1^2-a_2^2}{2a_1a_2}$, $\theta_1=\text{atan2}(y,x)-\text{atan2}(a_2s_2,a_1+a_2c_2)$; FK $x=a_1c_1+a_2c_{12}$, $y=a_1s_1+a_2s_{12}$.

## Velocity ([[05 Velocity Kinematics and Jacobian]])
- $S(\omega)=\begin{bmatrix}0&-\omega_z&\omega_y\\\omega_z&0&-\omega_x\\-\omega_y&\omega_x&0\end{bmatrix}$, $\dot R=S(\omega)R$, $v=\omega\times r$.
- $\xi=[v;\omega]=J\dot q$; revolute column $\begin{bmatrix}z_{i-1}\times(o_n-o_{i-1})\\z_{i-1}\end{bmatrix}$, prismatic column $\begin{bmatrix}z_{i-1}\\0\end{bmatrix}$.
- Planar 2R: $\det J_v=a_1a_2\sin\theta_2$; singular at $\theta_2=0,\pi$. Statics $\tau=J^TF$.
- $\dot q=J^{-1}\xi$ (square) · $J^T(JJ^T)^{-1}\xi$ (redundant) · $(J^TJ)^{-1}J^T\xi$ (over-determined); null space $(I-J^+J)b$.

## Trajectories ([[06 Trajectory Planning]])
- Cubic ($t_A=0$): $c_0=q_A$, $c_1=v_A$, $c_2=\dfrac{3\Delta-(2v_A+v_B)T}{T^2}$, $c_3=\dfrac{-2\Delta+(v_A+v_B)T}{T^3}$.
- Quintic: $c_0=q_A,\ c_1=v_A,\ c_2=a_A/2$, then solve $3\times3$ for $c_3,c_4,c_5$:
$\begin{bmatrix}T^3&T^4&T^5\\3T^2&4T^3&5T^4\\6T&12T^2&20T^3\end{bmatrix}$.
- Cubic ⇒ accel. discontinuous; quintic ⇒ accel. continuous. Rest-to-rest quintic: $q_A+\Delta(10\tau^3-15\tau^4+6\tau^5)$.

## Dynamics ([[07 Manipulator Dynamics]])
- $D(q)\ddot q+C(q,\dot q)\dot q+g(q)=\tau$.
- $D=\sum\big(m_iJ_{v_i}^TJ_{v_i}+J_{\omega_i}^TR_iI_iR_i^TJ_{\omega_i}\big)$, $\ \mathcal P=-\sum m_i\mathbf g^Tr_{ci}$, $\ g_k=\partial\mathcal P/\partial q_k$.
- $c_{ijk}=\tfrac12\big(\partial_{q_i}d_{kj}+\partial_{q_j}d_{ki}-\partial_{q_k}d_{ij}\big)$, $\ C_{kj}=\sum_ic_{ijk}\dot q_i$.
- Box: $I_{xx}=\frac m{12}(b^2+c^2)$ etc. Rod: $\frac{ml^2}{12}$ (centre), $\frac{ml^2}3$ (end).
- 2R: $D=\begin{bmatrix}a+2b\cos q_2&d+b\cos q_2\\d+b\cos q_2&d\end{bmatrix}$, $b=m_2l_1l_{c2}$.

## Control ([[08 Control]])
- Laplace: $\mathcal L[\dot f]=sF-f(0)$; IVT $f(0)=\lim_{s\to\infty}sF$; FVT $f(\infty)=\lim_{s\to0}sF$ (poles of $sF$ in LHP).
- Feedback: $\dfrac{A}{1+AB}$; unity loop $\dfrac{CG}{1+CG}$.
- 1st order $K/s$ closed loop: $\frac1{Ts+1}$, $T=1/K$. 2nd order: $\omega_n=\sqrt{K/J}$, $\zeta=\dfrac B{2\sqrt{KJ}}$; poles $-\zeta\omega_n\pm\omega_n\sqrt{\zeta^2-1}$.
- $\%OS=100e^{-\pi\zeta/\sqrt{1-\zeta^2}}$, $t_s\approx4/(\zeta\omega_n)$.
- DC motor: $L\dot i+Ri=V-K_b\omega$, $\tau=K_mi$, $J\dot\omega+B\omega=\tau-\tau_l/n$.
 Reduced ($L\approx0$): $\dfrac{\Theta}{V}=\dfrac{K_m/R}{s[Js+B+K_bK_m/R]}$. Full model: **3 poles**.
- Reflected inertia: $J_m+J_L/n^2$; load torque at motor $\tau_l/n$.
- PD: $\theta=\dfrac{(K_p+K_ds)\theta^d-D}{Js^2+(B+K_d)s+K_p}$, $e_{ss}=D/K_p$, $K_p=J\omega^2$, $K_d=2\zeta\omega J-B$.
- PID: stable iff $K_i<\dfrac{(B+K_d)K_p}J$; $e_{ss}=0$.
- Feedforward $F=1/G$ must be stable and proper. P on a DC motor with load: $e_{ss}=\dfrac{R\tau_l}{nK_iK_p}$.

## Vision ([[09 Sensors and Robot Vision]])
- $u=\lambda\dfrac xz$, $v=\lambda\dfrac yz$ (mm).
- $r=o_r-\alpha_xu$, $c=o_c-\alpha_yv$ (exam-consistent), $\alpha=1/s$ [pix/mm].
- Camera ↔ world: $p^b=T^b_cp^c$.
- Size/depth: $\dfrac{L_{world}}{\Delta_{mm}}=\dfrac z\lambda$.
- Stereo: $z=\dfrac{\lambda B}{u_1-u_2}$.
- Vision pipeline: threshold → erode/dilate → features → (PBVS pose / IBVS features) → control.
