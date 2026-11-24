---
tags: [robotics, 34753, kinematics, rotations]
---
# 02 Rotations and Homogeneous Transformations
Sources: Lecture 2, Lecture 3 (repetition), Exercises week 2. Prev: [[01 Introduction to Robotics]] · Next: [[03 DH Convention and Forward Kinematics]]

## Notation 🟦
- $p^1$ = coordinates of point $p$ **in frame 1**. $R^0_1$ = orientation of frame 1 **described in frame 0**. $o^0_1$ = origin of frame 1 **seen from frame 0**.
- Position + orientation = **placement (pose)**.
- A *free vector* (e.g. velocity) is expressed in another frame **only with $R$** (no translation).

## Rotation matrix 🟦
Columns are the axes of frame 1 written in frame 0:
$$R^0_1 = \begin{bmatrix} x^0_1 & y^0_1 & z^0_1 \end{bmatrix} = \begin{bmatrix} x_0\!\cdot\!x_1 & x_0\!\cdot\!y_1 & x_0\!\cdot\!z_1 \\ y_0\!\cdot\!x_1 & y_0\!\cdot\!y_1 & y_0\!\cdot\!z_1 \\ z_0\!\cdot\!x_1 & z_0\!\cdot\!y_1 & z_0\!\cdot\!z_1 \end{bmatrix}$$
Properties: orthonormal, $\det R = 1$, $R^{-1} = R^T$ ($R^1_0 = (R^0_1)^T$), $R_{z,\theta}R_{z,\phi} = R_{z,\theta+\phi}$, $R_{z,\theta}^{-1} = R_{z,-\theta}$.
9 entries, 6 constraints (3 unit-length + 3 orthogonality) → **3 independent parameters**.

### The three interpretations of $R$
1. **Orientation** of frame 1 w.r.t. frame 0: $R^0_1 = [x^0_1\,|\,y^0_1\,|\,z^0_1]$.
2. **Coordinate transformation** of a fixed point: $p^0 = R^0_1 p^1$, $\;p^1 = (R^0_1)^{-1}p^0$.
3. **Operator** rotating a point in the same frame: $p^0_b = R\,p^0_a$.

### Basic rotations
$$R_{x,\theta}=\begin{bmatrix}1&0&0\\0&c\theta&-s\theta\\0&s\theta&c\theta\end{bmatrix},\quad
R_{y,\theta}=\begin{bmatrix}c\theta&0&s\theta\\0&1&0\\-s\theta&0&c\theta\end{bmatrix},\quad
R_{z,\theta}=\begin{bmatrix}c\theta&-s\theta&0\\s\theta&c\theta&0\\0&0&1\end{bmatrix}$$
> [!tip] 🎯 Memory trick
> For $R_x$ and $R_z$ the active 2×2 block is $\begin{bmatrix}c&-s\\s&c\end{bmatrix}$ (minus top-right). For $R_y$ the block is **flipped**, $\begin{bmatrix}c&s\\-s&c\end{bmatrix}$ (minus bottom-left), because the pair is ordered $(z,x)$, not $(x,z)$. Sanity check: $R_{z,90°}$ maps $[1,0,0]^T\to[0,1,0]^T$ and $R_{y,90°}$ maps $[0,0,1]^T\to[1,0,0]^T$.

## Composition rule (the #1 exam trap) 🟦
> [!important]
> - Rotation/translation about the **current (moving) frame** → **post-multiply**: $R = R_{prev}\,R_{new}$
> - Rotation/translation about the **fixed / world / global frame** → **pre-multiply**: $R = R_{new}\,R_{prev}$

Lecture example, $R_{y,45°}$ then $R_{z,45°}$:
- both about **current** axes: $R = R_{y,45}R_{z,45}=\begin{bmatrix}\tfrac12&-\tfrac12&\tfrac1{\sqrt2}\\\tfrac1{\sqrt2}&\tfrac1{\sqrt2}&0\\-\tfrac12&\tfrac12&\tfrac1{\sqrt2}\end{bmatrix}$
- second about the **fixed** $z$: $R' = R_{z,45}R_{y,45}=\begin{bmatrix}\tfrac12&-\tfrac1{\sqrt2}&\tfrac12\\\tfrac12&\tfrac1{\sqrt2}&\tfrac12\\-\tfrac1{\sqrt2}&0&\tfrac1{\sqrt2}\end{bmatrix}$

Five-step example: (1) θ about current x, (2) φ about current z, (3) α about fixed z, (4) β about current y, (5) δ about fixed x gives
$$R^0_5 = R_{x,\delta}\,R_{z,\alpha}\,R_{x,\theta}\,R_{z,\phi}\,R_{y,\beta}$$
(fixed ones go to the **left**, current ones to the **right**, in order of occurrence.)

## Parametrisation of rotations 🟦

### ZYZ Euler angles (current axes)
$R = R_{z,\phi}R_{y,\theta}R_{z,\psi}$
$$R=\begin{bmatrix}c\phi c\theta c\psi-s\phi s\psi & -c\phi c\theta s\psi-s\phi c\psi & c\phi s\theta\\ s\phi c\theta c\psi+c\phi s\psi & -s\phi c\theta s\psi+c\phi c\psi & s\phi s\theta\\ -s\theta c\psi & s\theta s\psi & c\theta\end{bmatrix}$$

> [!warning] ⚠️ `Atan2` argument order
> The slides/textbook use **Atan2(x, y) = atan(y/x)** (x first). MATLAB/Python use **atan2(y, x)**. Everything below is written in **MATLAB order**.

Solutions ($\sin\theta\neq0$):

| | $\theta\in(0,\pi)$ | $\theta\in(-\pi,0)$ |
|---|---|---|
| $\theta$ | `atan2( sqrt(1-r33^2), r33 )` | `atan2( -sqrt(1-r33^2), r33 )` |
| $\phi$ | `atan2( r23, r13 )` | `atan2( -r23, -r13 )` |
| $\psi$ | `atan2( r32, -r31 )` | `atan2( -r32, r31 )` |

**Gimbal lock / singularity:** at $\theta = 0$ or $\pi$ ($\sin\theta=0$) only $\phi\pm\psi$ is determined, infinitely many solutions.

### Roll–Pitch–Yaw (fixed axes XYZ)
$R = R_{z,\phi}R_{y,\theta}R_{x,\psi}$ (yaw $\phi$ about z, pitch $\theta$ about y, roll $\psi$ about x, all about the **fixed** axes)
$$R=\begin{bmatrix}c\phi c\theta & c\phi s\theta s\psi - s\phi c\psi & c\phi s\theta c\psi + s\phi s\psi\\ s\phi c\theta & s\phi s\theta s\psi + c\phi c\psi & s\phi s\theta c\psi - c\phi s\psi\\ -s\theta & c\theta s\psi & c\theta c\psi\end{bmatrix}$$
For $c\theta\neq0$:
$$\theta=\text{atan2}\big(-r_{31},\ \pm\sqrt{r_{11}^2+r_{21}^2}\big),\quad \phi=\text{atan2}(r_{21},r_{11}),\quad \psi=\text{atan2}(r_{32},r_{33})$$
If $\theta=90°$: only $\psi-\phi = \text{atan2}(r_{22}, r_{12})$ is defined. If $\theta=-90°$ ($270°$): only $\psi+\phi$ (from $\text{atan2}(r_{22},-r_{12})$).
(Used in the Stanford-arm exercise, see [[11 Exercise Solutions]].)

### Axis–angle 🟦📖
$R = R_{k,\theta}$ with unit axis $k$. 📖 Rodrigues: $R = I + \sin\theta\,S(k) + (1-\cos\theta)\,S(k)^2$ ($S$ = skew matrix, see [[05 Velocity Kinematics and Jacobian]]).

## Homogeneous transformations 🟦
Point in frame 1 → frame 0: $p^0 = R^0_1p^1 + o^0_1$. Stack into a 4×4:
$$H^0_1=\begin{bmatrix}R^0_1 & o^0_1\\ 0_{1\times3} & 1\end{bmatrix}=\begin{bmatrix}x^0_1&y^0_1&z^0_1&o^0_1\\0&0&0&1\end{bmatrix},\qquad \begin{bmatrix}p^0\\1\end{bmatrix}=H^0_1\begin{bmatrix}p^1\\1\end{bmatrix}$$
> [!tip] 🎯 "Read it off the columns"
> Columns 1–3 = the axes $x,y,z$ of the new frame expressed in the old frame; column 4 = its origin. This is how Q1.2 and the week-2 problems are solved *by observation*.

**Inverse** (do **not** invert numerically by hand):
$$(H^0_1)^{-1} = H^1_0=\begin{bmatrix}R^T & -R^T o\\ 0&1\end{bmatrix}$$

**Special cases:** pure translation $\begin{bmatrix}I&o\\0&1\end{bmatrix}$; pure rotation $\begin{bmatrix}R&0\\0&1\end{bmatrix}$.

**Basic homogeneous matrices**
$$\text{Trans}_{x,a}=\begin{bmatrix}1&0&0&a\\0&1&0&0\\0&0&1&0\\0&0&0&1\end{bmatrix},\; \text{Rot}_{x,\alpha}=\begin{bmatrix}1&0&0&0\\0&c\alpha&-s\alpha&0\\0&s\alpha&c\alpha&0\\0&0&0&1\end{bmatrix}$$
(similarly $\text{Trans}_{y,b}$, $\text{Trans}_{z,c}$, $\text{Rot}_{y,\beta}$, $\text{Rot}_{z,\gamma}$.)

**Composition:** same rule as rotations: moves in the **current** frame post-multiply, moves in the **world** frame pre-multiply.

### Worked example: exam Q1.1 🧮
Sequence: rot 60° about *current* x → trans 50 mm along *current* y → rot 40° about *global* z → trans 30 mm along *global* x.
$$T=\underbrace{\text{Trans}_{x,30}}_{\text{global}}\;\underbrace{\text{Rot}_{z,40°}}_{\text{global}}\;\underbrace{\text{Rot}_{x,60°}}_{\text{current}}\;\underbrace{\text{Trans}_{y,50}}_{\text{current}}$$
Result: columns $(0.766,0.643,0)$, $(-0.321,0.383,0.866)$, $(0.557,-0.663,0.5)$, origin $(13.93,\,19.15,\,43.30)$. Full solution in [[12 Past Exam Solutions]].

## Common mistakes
- Pre/post-multiplication swapped. Write the sequence, label each step "global" or "current", then build the product from the *middle outwards*.
- Using degrees in code that expects radians.
- Forgetting that translation "along the current y" is **after** rotating (it is expressed in the rotated frame), so it contributes $R\,[0,50,0]^T$ to the origin.
