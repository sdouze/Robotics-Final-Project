---
tags: [robotics, 34753, dynamics, euler-lagrange]
---
# 07 Manipulator Dynamics (Euler–Lagrange)
Sources: Lecture 6 (second half), Exercises week 6, Control exercises §7.5, Lecture 7.2 review. Prev: [[06 Trajectory Planning]] · Next: [[08 Control]]

Hierarchy: kinematics < statics < dynamics. Rigid bodies < flexible couplings < deformable bodies (the course uses rigid bodies).

## Energies 🟦
**Kinetic energy** of $n$ rigid links:
$$\mathcal K=\sum_{i=1}^n\Big(\tfrac12m_iv_{c_i}^Tv_{c_i}+\tfrac12\omega_i^T\mathcal I_i\,\omega_i\Big)=\tfrac12\dot q^TD(q)\dot q$$
with $v_{c_i}=J_{v,c_i}\dot q$ and $\omega_i=J_{\omega_i}\dot q$ ($J_{v,c_i}$ = Jacobian of the **centre of mass** of link $i$, see [[05 Velocity Kinematics and Jacobian]]).

**Inertia tensor**
- Local: $I_i=\begin{bmatrix}I_{xx}&I_{xy}&I_{zx}\\I_{xy}&I_{yy}&I_{yz}\\I_{zx}&I_{yz}&I_{zz}\end{bmatrix}$ about the centre of mass, axes parallel to frame $i$.
- In the world frame: $\mathcal I_i=R_i\,I_i\,R_i^T$.
- Box $a\times b\times c$ (mass $m$): $I_{xx}=\frac{m}{12}(b^2+c^2)$, $I_{yy}=\frac m{12}(a^2+c^2)$, $I_{zz}=\frac m{12}(a^2+b^2)$.
- 📖 Slender rod length $l$: $\frac{ml^2}{12}$ about its centre, $\frac{ml^2}3$ about its end (parallel-axis theorem $I=I_c+md^2$).

**Inertia matrix** ($n\times n$, symmetric, positive definite 📖):
$$D(q)=\sum_{i=1}^n\Big(m_iJ_{v,c_i}^TJ_{v,c_i}+J_{\omega_i}^TR_iI_iR_i^TJ_{\omega_i}\Big)$$
**Potential energy** (gravity vector $\mathbf g$ in the world frame, e.g. $(0,0,-g)^T$ or $(0,-g,0)^T$):
$$\mathcal P=-\sum_{i=1}^nm_i\,\mathbf g^Tr_{c_i}(q)$$

## Euler–Lagrange equations 🟦
Lagrangian $\mathcal L=\mathcal K-\mathcal P=\tfrac12\dot q^TD\dot q-\mathcal P(q)$. For $k=1\dots n$:
$$\frac{d}{dt}\frac{\partial\mathcal L}{\partial\dot q_k}-\frac{\partial\mathcal L}{\partial q_k}=\tau_k$$
$$\sum_{j}d_{kj}(q)\ddot q_j+\sum_{i,j}c_{ijk}(q)\dot q_i\dot q_j+g_k(q)=\tau_k,\qquad c_{ijk}=\frac12\Big(\frac{\partial d_{kj}}{\partial q_i}+\frac{\partial d_{ki}}{\partial q_j}-\frac{\partial d_{ij}}{\partial q_k}\Big)$$
- $c_{ijk}$ = **Christoffel symbols**. $g_k=\partial\mathcal P/\partial q_k$.
- Terms with $\dot q_i^2$ = **centrifugal**; terms with $\dot q_i\dot q_j$ ($i\neq j$) = **Coriolis**.

**Matrix form**
$$\boxed{D(q)\ddot q+C(q,\dot q)\dot q+g(q)=\tau}\qquad C_{kj}=\sum_ic_{ijk}\dot q_i,\quad g=[g_1\dots g_n]^T$$
📖 Properties: $D=D^T>0$; $\dot D-2C$ is skew-symmetric; friction is often added as $+B\dot q$ (control exercises write $D\ddot q+C\dot q+g=\tau-B\dot q$).

> [!tip] 🎯 Recipe for a derivation
> 1. DH frames → $T^0_i$. 2. Centres of mass $r_{c_i}$ (from $T^0_i$ and the COM offset). 3. $J_{v,c_i},J_{\omega_i}$. 4. $I_i$ and $R_i$. 5. Build $D$. 6. Differentiate $d_{ij}$ → $c_{ijk}$. 7. $\mathcal P$ → $g_k$. 8. Assemble the equations; check units and simple limits (e.g. $\dot q=0$: only $D\ddot q+g$ remain).

## Example A: planar 2R, general closed form 🟦📖
Notation from the control exercises: $a=I_1+I_2+m_1l_{c1}^2+m_2(l_1^2+l_{c2}^2)$, $b=m_2l_1l_{c2}$, $d=I_2+m_2l_{c2}^2$:
$$D=\begin{bmatrix}a+2b\cos q_2&d+b\cos q_2\\d+b\cos q_2&d\end{bmatrix},\quad C\dot q=\begin{bmatrix}-b\sin q_2\,(2\dot q_1\dot q_2+\dot q_2^2)\\b\sin q_2\,\dot q_1^2\end{bmatrix}$$
$$g=\begin{bmatrix}(m_1l_{c1}+m_2l_1)\,g\cos q_1+m_2l_{c2}\,g\cos(q_1+q_2)\\m_2l_{c2}\,g\cos(q_1+q_2)\end{bmatrix}\ \ (\text{gravity along }-y,\ q_1\text{ measured from the horizontal})$$

## Example B: week-6 exercise (planar 2R, $L\times0.1L\times0.1L$ bars, mass $m$) 🟦
Centre of mass at $L_c=L/2$. Moments of inertia about the centre: $I_{xx}=\frac{mL^2}{600}$, $I_{yy}=I_{zz}=\frac{101\,mL^2}{1200}$. Only $I_{zz}$ matters for planar motion.
$$D=\frac{mL^2}{1200}\begin{bmatrix}2002+1200\cos\theta_2&401+600\cos\theta_2\\401+600\cos\theta_2&401\end{bmatrix}$$
Christoffel symbols: $c_{121}=c_{211}=c_{221}=-\frac{mL^2}2\sin\theta_2$, $c_{112}=+\frac{mL^2}2\sin\theta_2$, all others 0.
Potential ($\mathbf g=(0,-g,0)$): $\mathcal P=mgL(1.5\sin q_1+0.5\sin(q_1+q_2))$ ⇒
$$g_1=mgL\big(1.5\cos q_1+0.5\cos(q_1+q_2)\big),\quad g_2=0.5\,mgL\cos(q_1+q_2)$$
Equations of motion:
$$\begin{aligned}d_{11}\ddot q_1+d_{12}\ddot q_2+c_{121}\dot q_1\dot q_2+c_{211}\dot q_1\dot q_2+c_{221}\dot q_2^2+g_1&=\tau_1\\ d_{21}\ddot q_1+d_{22}\ddot q_2+c_{112}\dot q_1^2+g_2&=\tau_2\end{aligned}$$
(🧮 I re-derived $d_{11}$, $d_{12}$, $d_{22}$ from $m(L^2+2L_c^2+2LL_c\cos\theta_2)+2I_{zz}$ etc. and they match.)

## Example C: spatial 2-link (vertical joint 1, horizontal joint 2, rods mass $m$, length $l$) 🟦
DH: $(q_1,0,0,\pi/2)$, $(q_2,l,0,0)$. Results:
$$D=\frac{ml^2}3\begin{bmatrix}4+s_2^2&-\tfrac32c_2\\-\tfrac32c_2&1\end{bmatrix},\qquad g=\frac{mgl}{2}\begin{bmatrix}0\\-s_2\end{bmatrix}\ \ (\mathcal P=\tfrac{mgl}2c_2)$$
$$\tau_1=\tfrac{ml^2}3(4+s_2^2)\ddot q_1-\tfrac{ml^2}2c_2\ddot q_2+\tfrac{2ml^2}3s_2c_2\dot q_1\dot q_2+\tfrac{ml^2}2s_2\dot q_2^2$$
$$\tau_2=-\tfrac{ml^2}2c_2\ddot q_1+\tfrac{ml^2}3\ddot q_2-\tfrac{ml^2}3s_2c_2\dot q_1^2-\tfrac{mgl}2s_2$$
Interpretation: $d_{12}\neq0$ → joints are **inertially coupled**; $d_{11}$ grows from $4ml^2/3$ to $5ml^2/3$ as link 2 swings out; joint 1 carries no gravity load (vertical axis); $\dot q_1^2$ gives a centrifugal torque on joint 2.
**Numerical check** ($m=2$, $l=3$, $q=(\pi/5,\pi/3)$, $\dot q=(3,7)$, $\ddot q=(4,9)$): $D=\begin{bmatrix}28.5&-4.5\\-4.5&6\end{bmatrix}$, $g=(0,-25.49)$; $C\dot q+g=(491.04,-48.87)$; full $\tau=(564.54,-12.87)$ Nm. 🧮 ($D\ddot q=(73.5,36)$ ✔ adds up.)

## Motor inertia & gears (bridge to control) 🟦
Rotor with gear ratio $r$ ($\dot\theta_m=r\dot q$): $\mathcal K_m=\tfrac12J_m r^2\dot q^2$, so
$$\tilde D(q)=D(q)+\text{diag}(J_{m1}r_1^2,\ J_{m2}r_2^2)$$
Large $r$ → $\tilde D$ **diagonally dominant** → joints behave as decoupled SISO systems ([[08 Control]], independent joint control).

## Self-check
- Which terms vanish at rest? → $C\dot q$ (velocity terms).
- Why does $D$ depend on $q$? → Jacobians (and $R_i$) depend on configuration.
- Units of $c_{ijk}\dot q_i\dot q_j$? → torque (N·m).
