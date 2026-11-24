---
tags: [robotics, 34753, kinematics, DH]
---
# 03 DH Convention and Forward Kinematics
Sources: Lecture 3, Lecture 4 (repetition), Exercises week 3. Prev: [[02 Rotations and Homogeneous Transformations]] · Next: [[04 Inverse Kinematics]]

## Kinematic chain 🟦
- $n$ joints → $n$ DoF, $n+1$ links, $n+1$ frames $\{0\}\dots\{n\}$. Joint $i$ moves link $i$ (frame $\{i\}$ is rigidly attached to link $i$).
- Joint variables $q_i$ ($\theta_i$ revolute, $d_i$ prismatic). Single-link transform $A_i(q_i)=T^{i-1}_i$.
- **Forward kinematics:** $$H = T^0_n = A_1(q_1)\,A_2(q_2)\cdots A_n(q_n)$$
- Each $A_i$ built by hand is messy → use the **Denavit–Hartenberg (DH)** convention.

## The DH transformation 🟦
$$A_i=\text{Rot}_{z,\theta_i}\,\text{Trans}_{z,d_i}\,\text{Trans}_{x,a_i}\,\text{Rot}_{x,\alpha_i}
=\begin{bmatrix}c\theta_i&-s\theta_i c\alpha_i&s\theta_i s\alpha_i&a_i c\theta_i\\ s\theta_i&c\theta_i c\alpha_i&-c\theta_i s\alpha_i&a_i s\theta_i\\ 0&s\alpha_i&c\alpha_i&d_i\\0&0&0&1\end{bmatrix}$$

| Parameter | Name | Meaning (frame $i-1\to i$) |
|---|---|---|
| $\theta_i$ | joint angle | angle **from $x_{i-1}$ to $x_i$ about $z_{i-1}$** |
| $d_i$ | link offset | distance **from $o_{i-1}$ to the $x_i$-axis along $z_{i-1}$** |
| $a_i$ | link length | distance **from $z_{i-1}$ to $o_i$ along $x_i$** |
| $\alpha_i$ | link twist | angle **from $z_{i-1}$ to $z_i$ about $x_i$** |

> [!tip] 🎯 Mnemonic **θ – d – a – α** = "about – along – along – about" (about $z_{i-1}$, along $z_{i-1}$, along $x_i$, about $x_i$).

- **One of $\theta_i$ / $d_i$ is the variable** (marked $*$): $\theta_i$ for revolute, $d_i$ for prismatic. The other three are **constant** characteristics of the link.
- **Why 4 parameters, not 6?** Two DH conditions fix the frame placement:
  - **DH1:** $x_i \perp z_{i-1}$
  - **DH2:** $x_i$ intersects $z_{i-1}$

## Frame-assignment recipe 🟦📖
1. Number the joints $1\dots n$; link $i$ is moved by joint $i$.
2. Choose base frame $\{0\}$ with $z_0$ along joint-1's axis (any convenient $x_0, y_0$).
3. **$z_i$ along the axis of joint $i+1$** (so $z_{i-1}$ is the axis of joint $i$). Pick the direction freely.
4. **Origin $o_i$** on $z_i$ at the point of shortest distance to $z_{i-1}$ (where the common normal meets $z_i$).
5. **$x_i$ parallel to $z_i\times z_{i-1}$** (along the common normal), $y_i$ from the right-hand rule ($y_i = z_i\times x_i$).
6. Special cases 📖:
   - $z_{i-1}\parallel z_i$: origin $o_i$ is free along $z_i$ → place it so that $d_i = 0$ (or to make the next parameters simple); $x_i$ along the common normal.
   - $z_{i-1}$ and $z_i$ **intersect**: $o_i$ at the intersection, $a_i=0$, $x_i=\pm z_{i-1}\times z_i$ (sign is a free choice).
7. **End-effector frame $\{n\}$** (not a DH frame): origin between the fingers; $z_n$ = **a**pproach (along fingers); $y_n$ = **s**liding/closing direction; $x_n = y_n\times z_n$ = **n**ormal.
8. Fill the table by reading each parameter with the definitions above.

> [!warning] 🎯 Sign traps when reading a picture
> - $a_i$ and $d_i$ are **signed**: measure along the *positive direction of $x_i$ / $z_{i-1}$*. If $x_i$ points back toward the base, $a_i$ is negative (Q2.2 has $a_2=-400$).
> - $\alpha_i$: use the **right-hand rule about $x_i$**, turning $z_{i-1}$ onto $z_i$.
> - $\theta_i$: if $x_i$ is opposite to $x_{i-1}$ in the pictured pose, $\theta_i=180°$ (an offset), even though the joint "variable" is $\theta_i$.
> - Quick check: compute the column vectors of your $A_i$ and see whether they point where the picture says.

## Worked DH tables 🟦 (angles in degrees, lengths in mm)

**Three-link cylindrical (RPP)**
| $i$ | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ |
|---|---|---|---|---|
| 1 | $\theta_1^*$ | $d_1$ | 0 | 0 |
| 2 | 0 | $d_2^*$ | 0 | −90 |
| 3 | 0 | $d_3^*$ | 0 | 0 |

**SCARA (RRPR)**
| $i$ | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ |
|---|---|---|---|---|
| 1 | $\theta_1^*$ | $d_1$ | $a_1$ | 0 |
| 2 | $\theta_2^*$ | 0 | $a_2$ | 180 |
| 3 | 0 | $d_3^*$ | 0 | 0 |
| 4 | $\theta_4^*$ | $d_4$ | 0 | 0 |

**KUKA KR 210** (as printed on the slide)
| $i$ | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ |
|---|---|---|---|---|
| 1 | $\theta_1^*=180$ | 200 | 100 | 90 |
| 2 | $\theta_2^*=60$ | 0 | 300 | 0 |
| 3 | $\theta_3^*=-150$ | 0 | 20 | −90 |
| 4 | 90 | $d_4^*=300$ | 0 | 90 |
| 5 | $\theta_5^*=-45$ | 0 | 0 | −90 |
| 6 | $\theta_6^*=90$ | 50 | 0 | 0 |

> [!note] ⚠️ On the slide, row 4 marks $d_4$ (not $\theta_4$) with the star, i.e. a *prismatic* joint 4, unusual for a real KR 210. I copied it as printed; just don't assume the other rows' pattern.

**PUMA 260**
| $i$ | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ |
|---|---|---|---|---|
| 1 | $\theta_1^*$ | 350 | 0 | −90 |
| 2 | $\theta_2^*$ | 0 | 200 | 0 |
| 3 | $\theta_3^*$ | −50 | 0 | 0 |
| 4 | $\theta_4^*$ | 210 | 0 | 90 |
| 5 | $\theta_5^*$ | 0 | 0 | 90 |
| 6 | $\theta_6^*$ | 30 | 0 | 0 |

**Stanford arm** (week 3 exercise, $d_2=0.1$ m, $d_3$ prismatic; joints 4–6 not all visible, deduce from the $z$-axes)
| $i$ | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ |
|---|---|---|---|---|
| 1 | $\theta_1^*$ | 0 | 0 | −90 |
| 2 | $\theta_2^*$ | 0.1 | 0 | 90 |
| 3 | 0 | $d_3^*$ | 0 | 0 |
| 4 | $\theta_4^*$ | 0 | 0 | −90 |
| 5 | $\theta_5^*$ | 0 | 0 | 90 |
| 6 | $\theta_6^*$ | 0 | 0 | 0 |

**Planar 2R** (links 1 m, week 3 problem 2): $(\theta_1^*,0,1,0)$, $(\theta_2^*,0,1,0)$ →
$$T^0_2=\begin{bmatrix}c_{12}&-s_{12}&0&c_1+c_{12}\\s_{12}&c_{12}&0&s_1+s_{12}\\0&0&1&0\\0&0&0&1\end{bmatrix}$$

## Reading a robot picture (exam Q2.1 skills) 🟦🧮
- **Count joints** = number of motors/axes (each has its own axis $z$). A UR-style or KUKA-style arm has **6 revolute, 0 prismatic**.
- **Which joint does a motor drive?** The motor sits on link $i-1$ and rotates link $i$ → it drives **joint $i$**. A motor on the turret with a horizontal axis drives the shoulder = **joint 2**; a vertical motor/gear in the base turns joint 1.
- **Link numbering:** link 0 = fixed base, link 1 = what joint 1 turns (turret), link 2 = upper arm, link 3 = forearm/elbow housing, link 4 = first wrist element, … link 6 = flange.
- Worked answers for Q2.1 and the full Q2.2 table are in [[12 Past Exam Solutions]].

## Forward kinematics in practice 📖
1. Fill the DH table. 2. Build each $A_i$ (MATLAB/Python function). 3. Multiply left to right. 4. Read position from column 4, orientation from the $3\times3$ block. Check against a hand sketch of a simple pose ($q=0$, $q=90°$).

## Self-check
- $T^0_n$ columns 1–3: $n$ (normal), $s$ (sliding), $a$ (approach) of the tool; column 4: tool position. ✔
- Forward kinematics is always **unique**; inverse kinematics is **not** → [[04 Inverse Kinematics]].
