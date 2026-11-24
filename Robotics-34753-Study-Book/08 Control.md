---
tags: [robotics, 34753, control, DC-motor, PID]
---
# 08 Control: Laplace, Transfer Functions, PID, DC Motor, Independent Joint Control
Sources: Lecture 7.1 (recap of linear control), Lecture 7.2 (independent joint control), Control exercises (Fall 2025), motor exercise, Reflected-inertia note, Exam Q5.1–5.2. Prev: [[07 Manipulator Dynamics]] · Next: [[09 Sensors and Robot Vision]]

## 1. Feedback basics 🟦
- **Open loop:** controller drives the process directly, no measurement. **Closed loop:** the output is measured and compared with the reference.
- **Pros of feedback:** rejects disturbances, more accurate (even zero steady-state error), response time can be shaped. **Con:** possible **instability** (sustained oscillation).
- Design steps: choose how to actuate; pick sensors; set requirements (steady-state accuracy, response time); analyse stability; modify until the requirements hold.
- Robot feedback loop: *reference trajectory → compensator → power amplifier → (+disturbance) → plant → output → sensor → back to the sum*. Design goal: choose the **compensator** so that the plant's output **follows** the reference.

## 2. Laplace transform 🟦
$$F(s)=\mathcal L[f(t)](s)=\int_0^{\infty}f(t)e^{-st}\,dt,\quad s\in\mathbb C$$
| $f(t)$ | $F(s)$ |
|---|---|
| impulse | 1 |
| step | $1/s$ |
| $\cos\omega t$ | $s/(s^2+\omega^2)$ |
| $\sin\omega t$ | $\omega/(s^2+\omega^2)$ |
| $e^{at}$ | $1/(s-a)$ |

- Linearity: $\mathcal L[\alpha_1f_1+\alpha_2f_2]=\alpha_1F_1+\alpha_2F_2$.
- Derivatives: $\mathcal L[\dot f]=sF(s)-f(0)$, $\ \mathcal L[\ddot f]=s^2F(s)-sf(0)-\dot f(0)$. (Differential equations → algebraic equations.)
- **Inverse transform** of a strictly proper $F=N/D$ ($\deg D>\deg N$) by partial fractions (**Heaviside method**).
- **Initial value:** $f(0)=\lim_{s\to\infty}sF(s)$. **Final value:** $f(\infty)=\lim_{s\to0}sF(s)$.

> [!warning] ⚠️ Slide error (Lecture 7.1, p.13)
> The slide says the final value theorem holds "if the roots of $D(s)$ are equal to 0 or have real part **greater** than 0". That is **backwards**. The theorem holds when all poles of $sF(s)$ lie in the **open left half-plane** (real part < 0); a pole at the origin of $F$ itself (e.g. a step input) is fine because $s$ cancels it.

## 3. Transfer function (TF) 🟦
$Y(s)=G(s)U(s)$ for a linear system, **zero initial conditions**. $G(s)=C(sI-A)^{-1}B+D=N(s)/D(s)$.
- The TF of a linear system is the Laplace transform of its **unit-impulse response**.
- **Poles** = roots of $D(s)$ = eigenvalues of $A$ (a system of order $n$ has $n$ poles). **Zeros** = roots of $N(s)$.
- Canonical (Bode) form
$$G(s)=\frac{\mu}{s^g}\frac{\prod_i(1+T_is)\prod_i\big(1+2\frac{\zeta_i}{\sigma_{ni}}s+\frac{s^2}{\sigma_{ni}^2}\big)}{\prod_i(1+\tau_is)\prod_i\big(1+2\frac{\xi_i}{\omega_{ni}}s+\frac{s^2}{\omega_{ni}^2}\big)}$$
 $\mu$ = gain, **$g$ = type** (number of poles at the origin if $g>0$; number of zeros at 0 if $g<0$), $T_i,\tau_i$ = time constants, $\omega_{ni}$ = natural frequencies, $\xi_i,\zeta_i\in(-1,1)$ = damping coefficients of the complex pairs.
- Example R-L-C filter (output across $C$): $\dfrac{E_o}{E_i}=\dfrac1{LCs^2+RCs+1}$.

## 4. Block-diagram algebra 🟦
| Connection | Equivalent |
|---|---|
| series $A,B$ | $AB$ |
| parallel $A,B$ (summed) | $A+B$ |
| feedback, negative | $\dfrac{A}{1+AB}$ |
| feedback, positive | $\dfrac{A}{1-AB}$ |
Unity-feedback loop with controller $C$ and plant $G$: $\displaystyle T(s)=\frac{Y}{R}=\frac{CG}{1+CG}$.
Example from the slides: minor loop $G_1$ with feedback $H_1$ and gain $K$ → $\dfrac{KG_1}{1+KG_1H_1}$, cascaded with $G_2$ → $G_{equiv}=\dfrac{KG_1G_2}{1+KG_1H_1}$ in a unity loop.

## 5. First- and second-order systems 🟦
**First order:** $K/s$ in unity feedback → $\dfrac CR=\dfrac1{sT+1}$, $T=\dfrac1K$ (time constant), pole at $s=-K$ → stable for $K>0$.

**Second order:** $\dfrac CR=\dfrac K{Js^2+Bs+K}=\dfrac{\omega_N^2}{s^2+2\xi\omega_Ns+\omega_N^2}$ with
$$\xi=\frac B{2\sqrt{KJ}}\ \ (\text{damping ratio}),\qquad\omega_N=\sqrt{K/J}\ \ (\text{undamped natural frequency})$$
⚠️ (slide prints the denominator without the $s$ in $2\xi\omega_N s$; it's a typo.)
- Poles: $s=-\zeta\omega_n\pm\omega_n\sqrt{\zeta^2-1}$.
- $\zeta<1$: underdamped (oscillation, overshoot) · $\zeta=1$: critically damped · $\zeta>1$: overdamped (slow, no overshoot). $\omega_n$ larger → faster.

**Response metrics** 🟦 (definitions): **overshoot** = % of final value exceeded at the first peak; **rise time** = time from 10 % to 90 % of final value; **settling time** = time to stay within 2 % of final value; **steady-state error** = difference from desired value.
📖 Approximations: $\%OS=100\,e^{-\pi\zeta/\sqrt{1-\zeta^2}}$, $t_s\approx4/(\zeta\omega_n)$, $t_r\approx1.8/\omega_n$.

**Stability (s-plane)** 🟦: poles in the **left half-plane → asymptotically stable**; on the imaginary axis → **marginally stable** (sustained oscillation); right half-plane → **unstable**.

## 6. Controllers 🟦
| | Law | TF | Remarks |
|---|---|---|---|
| P | $u=K_pe$ | $K_p$ | adjustable gain |
| I | $u=K_i\!\int_0^t e\,dt$ | $K_i/s$ | **eliminates steady-state error (bias)**, may cause oscillations |
| D | $u=K_d\dot e$ | $K_ds$ | effective in transients, faster response, **never used alone** |
I and D are used **in combination with at least P**. PID: $C(s)=K_p+\dfrac{K_i}s+K_ds$.

## 7. Independent joint model & DC motor 🟦
**Permanent-magnet DC motor:** torque $\tau_m=K_1\phi i_a=K_mi_a$; back-emf $V_b=K_2\phi\omega_m=K_b\omega_m$; electrical $L\dfrac{di_a}{dt}+Ri_a=V-V_b$.
**Single link with gear train** (gear ratio $r$: $\theta_m=r\theta_s$ where $\theta_s$ = load side):
$$J_m\ddot\theta_m+B_m\dot\theta_m=\tau_m-\tau_l/r=K_mi_a-\tau_l/r,\qquad J_m=J_a+J_g$$
Laplace: $(J_ms^2+B_ms)\Theta_m=K_mI_a-\tau_l/r$ and $(Ls+R)I_a=V-K_bs\Theta_m$. Block diagram: $V\to\frac1{Ls+R}\to I_a\to K_m\to(+)\ -\tau_l/r\ \to\frac1{J_ms+B_m}\to\omega_m\to\frac1s\to\theta_m$, with $K_b$ feeding $\omega_m$ back to the first sum.

$$\frac{\Theta_m}{V}=\frac{K_m}{s\big[(Ls+R)(J_ms+B_m)+K_bK_m\big]}\ \ (\text{3rd order}),\qquad\frac{\Theta_m}{\tau_l}=\frac{-(Ls+R)/r}{s\big[(Ls+R)(J_ms+B_m)+K_bK_m\big]}$$
**Reduction when $L/R\approx0$** (electrical dynamics negligible) → 2nd order:
$$\boxed{\frac{\Theta_m}{V}=\frac{K_m/R}{s\big[J_ms+B_m+K_bK_m/R\big]}},\qquad\frac{\Theta_m}{\tau_l}=\frac{-1/r}{s\big[J_ms+B_m+K_bK_m/R\big]}$$
- The back-emf adds **electrical damping** $K_bK_m/R$ to the mechanical damping $B_m$.
- Poles of the full model: $s=0$ (integrator $\theta=\int\omega$) + 2 from the quadratic. **Non-negligible $L$ ⇒ 3 poles.** Neglecting $L$ ⇒ 2 poles.

**Reflected inertia** 🟦 (handwritten note): the load inertia seen at the motor shaft is $J_L/n^2$:
$$\tau_{Lm}=\tfrac1{n^2}J_L\ddot\theta_m+\tfrac1n\tau_L\ \Rightarrow\ \Big(J_m+\frac{J_L}{n^2}\Big)\ddot\theta_m+D_m\dot\theta_m=\tau_m-\frac1n\tau_L$$
Bigger gear ratio $n$ → smaller reflected inertia (and smaller disturbance $\tau_L/n$).

### Why independent joint control works 🟦
With motor inertias, $\tilde D(q)=D(q)+\text{diag}(J_{mi}r_i^2)$. As $r_i$ grows the diagonal dominates the off-diagonal coupling and the Coriolis term gets relatively small, so each joint can be controlled as a **SISO** system. The ignored coupling torque is
$$\tau_{cpl}=\big(\tilde D-\text{diag}\tilde D\big)\ddot q_{cmd}+C(q,\dot q)\dot q$$
Simulation (2-link arm, $r=20$ vs $r=100$): at $r=20$ the independent controller shows big initial transients and larger torque peaks; at $r=100$ it overlays the full computed-torque controller and $\|\tau_{cpl}\|$ is tiny.
Full model-based torque: $\tau=\tilde D(q)v+C\dot q+g+B\dot q$. Independent joint torque: $\tau=\text{diag}(\tilde D)\ddot q_{cmd}+g+B\dot q$.

## 8. Set-point tracking with PD 🟦
Plant (per joint, torque input): $\dfrac1{Js+B}\cdot\dfrac1s$. Controller $U=(K_p+K_ds)(\theta^d-\theta)$, disturbance torque $D$:
$$\theta=\frac{(K_p+K_ds)\theta^d(s)-D(s)}{Js^2+(B+K_d)s+K_p}$$
- Open loop has one $1/s$ → **type I**.
- Characteristic equation $Js^2+(B+K_d)s+K_p=0$, roots $s=\dfrac{-(B+K_d)\pm\sqrt{(B+K_d)^2-4JK_p}}{2J}$.
- **Stable for all positive $K_p,K_d$** (complex roots, double root or two negative real roots).
- **Steady-state error** for step reference and constant disturbance (final value theorem):
$$e_{ss}=\lim_{s\to0}sE(s)=\frac{D}{K_p}$$
Since $D\propto1/r$ (torque at the motor), **larger gear ratio ⇒ smaller $e_{ss}$**.
- **Design** (match $s^2+\frac{B+K_d}Js+\frac{K_p}J$ with $s^2+2\zeta\omega s+\omega^2$): $$K_p=J\omega^2,\qquad K_d=2\omega J\zeta-B$$ Usually $\zeta=1$ (critical damping).

## 9. PID 🟦
$U=(K_p+K_ds+K_i/s)(\theta^d-\theta)$, $K_p,K_d,K_i>0$.
$$\theta(s)=\frac{(K_ds^2+K_ps+K_i)\theta^d-sD}{Js^3+(B+K_d)s^2+K_ps+K_i}\quad(\text{third order})$$
- **Routh–Hurwitz:** stable iff $K_i<\dfrac{(B+K_d)K_p}{J}$ 📖 (all coefficients positive and $(B+K_d)K_p>JK_i$).
- **$e_{ss}=0$** for constant disturbance (integrator).

## 10. Input saturation 🟦
Actuator limits (max/min torque or current). Example with $K_P=64,K_D=15,K_I=3.5$, disturbance 40, limits ±50: **much slower rise time**. 📖 With an integrator, saturation also causes *integrator windup*, so use anti-windup.

## 11. Feedforward for time-varying trajectories 🟦
Add $F(s)$ in parallel with the feedback $H(s)$ ($G=q/p$, $H=c/d$, $F=a/b$):
$$\frac\theta{\theta^d}=\frac{(bc+ad)\,q}{(pd+cq)\,b}$$
Stable if $pd+cq$ and $b$ are Hurwitz → **the feedforward TF itself must be stable (and proper)**. Choosing $F=1/G$ gives $\theta=\theta^d$ (zero tracking error); with a disturbance the steady-state error is determined only by the disturbance.
For the joint model $F(s)=Js^2+Bs$, i.e. $\tau_{ff}=J\ddot\theta^d+B\dot\theta^d$ (use the derivatives of the planned trajectory rather than differentiating numerically) 📖.

## 12. Worked numbers
- **A1** (rotor $J\dot\omega+b\omega=\tau$): $G_1=\dfrac1{Js+b}$, pole $s=-b/J$ ($=-4$ for $J=0.05$, $b=0.2$).
- **A2** (series RLC, $R=10$, $L=0.5$, $C=0.001$): $\omega_n=1/\sqrt{LC}=44.72$ rad/s, $\zeta=\frac R2\sqrt{C/L}=0.224$, poles $-10\pm43.59i$.
- **B1** (P on A1): $T_1=\dfrac{K_p}{Js+b+K_p}$, pole $-(b+K_p)/J$ → $-14,-24,-44,-404$ for $K_p=0.5,1,2,20$. Larger $K_p$ = faster, but still steady-state error (no integrator; DC gain $K_p/(b+K_p)<1$).
- **B2** (P on A2): $T_2=\dfrac{K_p}{LCs^2+RCs+1+K_p}$: larger $K_p$ pushes the imaginary part of the poles up (more oscillatory).
- **C1** (PI on A1): $T_1=\dfrac{K_ps+K_i}{Js^2+(b+K_p)s+K_i}$; **C2** (PI on A2, ⚠️ the exercise PDF repeats the P formula by mistake; the correct one is) $T_2=\dfrac{K_ps+K_i}{LCs^3+RCs^2+(1+K_p)s+K_i}$.
- **D1/D2** (PID): $T_1=\dfrac{K_ds^2+K_ps+K_i}{(J+K_d)s^2+(b+K_p)s+K_i}$, $T_2=\dfrac{K_ds^2+K_ps+K_i}{LCs^3+(RC+K_d)s^2+(1+K_p)s+K_i}$. $K_d$ changes the oscillatory behaviour massively.
- **E1** (loop with disturbance at the plant input, $C=k$, $P=\frac1{ms+b}$): $G_{yr}=\dfrac{k}{ms+b+k}$, $G_{yd}=\dfrac1{ms+b+k}$.
- **E2** (block reduction): $G_{yr}=\dfrac{0.5s^2+3s+4.5}{1.5s^2+9s+9.5}$. 🧮 (verified: inner loop → $\frac1{s+5}$; series with $G_1$: $\frac2{(s+1)(s+5)}$; add $G_3$; close unity loop.)
- **DC motor exercise** ($R=2$, $L=2$ mH, $K_m=K_b=0.08$, $J_m=2\cdot10^{-4}$, $B_m=2\cdot10^{-3}$, PID $0.12/2.2/0.01$): 🧮 reduced plant $\dfrac{0.04}{s(2\cdot10^{-4}s+0.0052)}=\dfrac{200}{s(s+26)}$; mechanical pole at $-26$ rad/s; electrical pole $\approx-R/L=-1000$ rad/s (hence the reduction is justified). Simulink model: `PID → Km/(La s+Ra) → 1/(J s+b) → 1/s`, with $K_b$ fed back from the speed to the first sum.

## Exam Q5.1 / Q5.2 methods
- **Number of poles** of the electromechanical model with non-negligible $L$: $\Theta_m/V$ has denominator $s[(Ls+R)(J_{eq}s+D)+K_bK_m]$ → **3 poles**. (If the output were speed $\omega_m$, there would be 2.) Full answer: [[12 Past Exam Solutions]].
- **Steady-state error with P control and a constant load torque:** in steady state $\dot\theta=0$, so no back-emf: $K_i\,i=\tau_l/n$ and $V=Ri=K_pe$ →
$$e_{ss}=\frac{R\,\tau_l}{n\,K_i\,K_p}$$
- Always identify parameters by **units**: $K_m$ [V·s/rad] (back-emf) vs $K_i$ [N·m/A] (torque const.), $J$ [kg·m²], $D_m$ [kg·m²/s], $n$ gear ratio.

## Self-check
- Why can't you use D alone? → it doesn't act on constant errors and amplifies noise.
- What does I do to type? → adds a pole at the origin, giving zero steady-state error to constant disturbances.
- When can the DC motor be approximated by 2nd order? → $L/R\approx0$ (electrical time constant much faster than mechanical).
