---
tags: [robotics, 34753, trajectory]
---
# 06 Trajectory Planning
Sources: Lecture 6 (first half), Lecture 7.2 review slides, Exam Q3.2. Prev: [[05 Velocity Kinematics and Jacobian]] · Next: [[07 Manipulator Dynamics]]

## Path vs trajectory 🟦
- **Path:** purely geometric route (points in configuration space or workspace), no time.
- **Trajectory:** a path **with time**: $q(t)$ and its derivatives $\dot q(t),\ddot q(t)$.
- Planning chain: Task plan → Action plan → **Path plan** (geometry, obstacle avoidance, shortest path) → **Trajectory plan** (interpolate with polynomials in time; satisfy position, velocity, acceleration constraints) → Controller → Robot ← Sensors.
- The planner works in **joint space** $q(t),\dot q,\ddot q$ or **Cartesian space** $p(t),v(t),a(t)$, subject to path constraints (smoothness).
- Define $q(t)$ **piecewise** (segments $t_A\to t_B\to t_C\dots$), prescribing $q,\dot q,\ddot q$ at each knot.

## Continuity 🟦
Derivatives of position: velocity $\dot q=v$, acceleration $\ddot q=a$, jerk $\dddot q=j$.
- If velocity jumps at $t_B$ → **acceleration = ∞** there.
- If acceleration jumps at $t_B$ → **jerk = ∞**.
- **Always avoid infinite acceleration and jerk** (excites vibrations, saturates motors).
- Naive **linear interpolation** $q(t)=\frac{t_B-t}{t_B-t_A}s_A+\frac{t-t_A}{t_B-t_A}s_B=c_0+c_1t$ cannot match velocities (velocity jumps at the ends), and a straight line in *joint* space is generally a **curve** in the workspace.

## Cubic interpolation (4 conditions) 🟦
$$q(t)=c_0+c_1t+c_2t^2+c_3t^3,\qquad \dot q(t)=c_1+2c_2t+3c_3t^2$$
Satisfies 2 initial + 2 final conditions: $q(t_A)=q_A,\ \dot q(t_A)=v_A,\ q(t_B)=q_B,\ \dot q(t_B)=v_B$:
$$\begin{bmatrix}1&t_A&t_A^2&t_A^3\\0&1&2t_A&3t_A^2\\1&t_B&t_B^2&t_B^3\\0&1&2t_B&3t_B^2\end{bmatrix}\begin{bmatrix}c_0\\c_1\\c_2\\c_3\end{bmatrix}=\begin{bmatrix}q_A\\v_A\\q_B\\v_B\end{bmatrix}$$
**Closed form** 📖🧮 for $t_A=0$, $T=t_B$: $c_0=q_A$, $c_1=v_A$,
$$c_2=\frac{3(q_B-q_A)-(2v_A+v_B)T}{T^2},\qquad c_3=\frac{-2(q_B-q_A)+(v_A+v_B)T}{T^3}$$
Acceleration of a cubic is **linear in $t$** and generally **discontinuous** at the ends (see the review slides: cubic → "discontinuity in the acceleration").

### Lecture example (soccer-ball catcher, $t_A=0,\ t_B=1$ s) 🟦
Start: $q(0)=(0.25\pi,\ 0.6\pi,\ -0.52\pi,\ 0)$, $\dot q(0)=0$. End: $q(1)=(0.33\pi,\ 0.4\pi,\ -0.4\pi,\ -0.2\pi)$, $\dot q(1)=(3.60,\ 9.25,\ -7.75,\ -5.10)$.

| joint | $c_0$ | $c_1$ | $c_2$ | $c_3$ |
|---|---|---|---|---|
| 1 | 0.785 | 0 | −2.842 | 3.093 |
| 2 | 1.885 | 0 | −11.131 | 10.503 |
| 3 | −1.634 | 0 | 8.877 | −8.500 |
| 4 | 0 | 0 | 3.211 | −3.839 |

## Quintic interpolation (6 conditions) 🟦
$$q=c_0+c_1t+c_2t^2+c_3t^3+c_4t^4+c_5t^5$$
$$\dot q=c_1+2c_2t+3c_3t^2+4c_4t^3+5c_5t^4,\qquad \ddot q=2c_2+6c_3t+12c_4t^2+20c_5t^3$$
3 initial + 3 final conditions ($q,\dot q,\ddot q$ at $t_A$ and $t_B$):
$$\begin{bmatrix}1&t_A&t_A^2&t_A^3&t_A^4&t_A^5\\0&1&2t_A&3t_A^2&4t_A^3&5t_A^4\\0&0&2&6t_A&12t_A^2&20t_A^3\\1&t_B&t_B^2&t_B^3&t_B^4&t_B^5\\0&1&2t_B&3t_B^2&4t_B^3&5t_B^4\\0&0&2&6t_B&12t_B^2&20t_B^3\end{bmatrix}\begin{bmatrix}c_0\\c_1\\c_2\\c_3\\c_4\\c_5\end{bmatrix}=\begin{bmatrix}q_A\\v_A\\a_A\\q_B\\v_B\\a_B\end{bmatrix}$$
Quintic gives continuous acceleration (finite jerk): smooth start and stop (review-slide plots: cubic accel. is a line with jumps, quintic accel. is a smooth curve starting and ending near 0).

> [!tip] 🎯 Shortcut when $t_A=0$
> The first three rows give $c_0=q_A,\ c_1=v_A,\ c_2=a_A/2$ immediately. Only a **3×3 system** in $c_3,c_4,c_5$ remains:
> $$\begin{bmatrix}T^3&T^4&T^5\\3T^2&4T^3&5T^4\\6T&12T^2&20T^3\end{bmatrix}\begin{bmatrix}c_3\\c_4\\c_5\end{bmatrix}=\begin{bmatrix}q_B-c_0-c_1T-c_2T^2\\v_B-c_1-2c_2T\\a_B-2c_2\end{bmatrix}$$

### Exam Q3.2 🧮
$t_A=0,\ q_A=\pi/4,\ v_A=a_A=0$; $T=t_B=2$ s, $q_B=0,\ v_B=1,\ a_B=0$.
$c_0=\pi/4=0.7854$, $c_1=c_2=0$ and
$$\begin{bmatrix}8&16&32\\12&32&80\\12&48&160\end{bmatrix}\begin{bmatrix}c_3\\c_4\\c_5\end{bmatrix}=\begin{bmatrix}-\pi/4\\1\\0\end{bmatrix}\ \Rightarrow\ c_3=-1.982,\ c_4=1.611,\ c_5=-0.3348$$
(also in [[12 Past Exam Solutions]]).

## Extras 📖
- **Rest-to-rest quintic** ($v=a=0$ at both ends): $q(t)=q_A+\Delta\,(10\tau^3-15\tau^4+6\tau^5)$, $\tau=t/T$, $\Delta=q_B-q_A$. Peak speed $1.875\,\Delta/T$ at mid-time.
- **Rest-to-rest cubic:** $q=q_A+\Delta(3\tau^2-2\tau^3)$, peak speed $1.5\,\Delta/T$.
- **Trapezoidal velocity profile (LSPB):** constant acceleration → constant velocity → constant deceleration; velocity is continuous but acceleration jumps. 
- The lecture overview also lists a **constant-jerk curve** (piecewise-cubic accel. profile) as a way to bound jerk. The slides don't go deeper.
- **Via points:** pass a velocity at the knot (e.g. the Jacobian-based $\dot q$ in the ball-catcher example) so segments join smoothly.

## Joint space vs Cartesian space
| | Joint-space interpolation | Cartesian-space interpolation |
|---|---|---|
| Cost | cheap, no IK along the path | needs IK (and Jacobian) at every step |
| Tip path | generally curved | straight line (or desired path) |
| Singularities | avoided automatically | must be checked |

## Self-check
- How many conditions does a polynomial of order $n$ satisfy? → $n+1$ coefficients, so 4 for cubic, 6 for quintic.
- Which polynomial order do you need to prescribe start/end *acceleration*? → quintic.
