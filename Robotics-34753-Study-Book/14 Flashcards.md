---
tags: [robotics, 34753, flashcards]
---
# 14 Flashcards
Format `Question::Answer` (single line), compatible with the Obsidian *Spaced Repetition* plugin (set the plugin's flashcard tag to `#flashcards`). You can also just cover the right side. Prev: [[13 Formula Sheet]] · Home: [[DTU/Semester 1/Robotics/Robotics-34753-Study-Book/00 Index]]

## Introduction #flashcards/intro
A serial arm with $n$ joints has how many links?::$n+1$ (including the fixed base).
Joint $i$ moves which link?::Link $i$ (and is attached to link $i-1$).
Variable of a revolute joint vs prismatic joint?::$\theta$ (angle) vs $d$ (displacement).
Minimum DoF for arbitrary position and orientation?::6.
DoF for a rotationally symmetric tool (e.g. welding)?::5.
Joint types of SCARA / Cylindrical / Cartesian / Articulated?::RRP / RPP / PPP / RRR. (Spherical is RRP.)
Reachable vs dexterous workspace?::Reachable: any orientation not required. Dexterous: reachable with arbitrary orientation.
What is a spherical wrist?::Three revolute axes intersecting in one point (the wrist centre).
Stages of the robot operational loop?::Sensing → planning → actuation → feedback.
Order of the course's analytical pipeline?::Kinematics → trajectory planning → dynamics → control → perception (vision).

## Rotations and transforms #flashcards/rotations
Properties of a rotation matrix?::Orthonormal, $\det=1$, $R^{-1}=R^T$, 3 independent parameters.
Rotation about the **current** frame: multiply on which side?::Right (post-multiply).
Rotation about the **world/fixed** frame: multiply on which side?::Left (pre-multiply).
What do the columns of $R^0_1$ represent?::The axes $x_1,y_1,z_1$ expressed in frame 0.
Inverse of a homogeneous transform?::$\begin{bmatrix}R^T&-R^To\\0&1\end{bmatrix}$.
Why do ZYZ and RPY have singularities?::Gimbal lock: at $\sin\theta=0$ (ZYZ) or $\cos\theta=0$ (RPY) two axes align and only a sum/difference of angles is defined.
Atan2 order in the slides vs MATLAB?::Slides `Atan2(x,y)=atan(y/x)`; MATLAB `atan2(y,x)`.
Is a velocity vector changed between fixed frames by $H$ or $R$?::By $R$ only (free vector).

## DH and FK #flashcards/dh
Four DH parameters and their meaning?::$\theta$: $x_{i-1}\to x_i$ about $z_{i-1}$; $d$: along $z_{i-1}$; $a$: along $x_i$; $\alpha$: $z_{i-1}\to z_i$ about $x_i$.
Which DH parameter is the variable for a revolute / prismatic joint?::$\theta_i$ / $d_i$.
Two DH conditions?::DH1: $x_i\perp z_{i-1}$. DH2: $x_i$ intersects $z_{i-1}$.
Where is $z_i$ placed?::Along the axis of joint $i+1$.
$x_i$ direction?::Along the common normal, parallel to $z_i\times z_{i-1}$.
What if $z_{i-1}\parallel z_i$?::Origin $o_i$ is free along $z_i$ (choose it to simplify, e.g. $d=0$).
What if $z_{i-1}$ and $z_i$ intersect?::$a_i=0$ and $x_i=\pm z_{i-1}\times z_i$.
End-effector frame letters $x_n,y_n,z_n$?::Normal, sliding, approach: $z_n$ approach, $y_n$ sliding, $x_n=y_n\times z_n$.
$a_2=-400$ in a figure means?::$o_2$ lies in the **negative** $x_2$ direction from $o_1$ (e.g. $x_2$ points toward the base).
Which joint does the motor mounted on link $i-1$ drive?::Joint $i$.

## Inverse kinematics #flashcards/ik
Why is IK harder than FK?::Nonlinear equations, multiple/no/infinite solutions.
Kinematic decoupling requirement?::Last three axes intersect in one point (spherical wrist).
Wrist centre formula?::$o_c=o_6-d_6R^0_6[0,0,1]^T$.
Law of cosines for a 2R arm?::$\cos\theta_2=\dfrac{x^2+y^2-a_1^2-a_2^2}{2a_1a_2}$.
$\theta_1$ of the planar 2R arm?::$\text{atan2}(y,x)-\text{atan2}(a_2s_2,a_1+a_2c_2)$.
What if $|\cos\theta_2|>1$?::Target unreachable.
Number of IK solutions of a 6R arm with spherical wrist?::Up to 8 (shoulder × elbow × wrist flip).
Inverse orientation after decoupling?::$R^3_6=(R^0_3)^TR^0_6$ → Euler-angle extraction.
Exam Q2.3: $a_1=391$, $a_2=302$, target $(0,592.7)$, elbow down?::$\theta_1=63°$, $\theta_2=63°$.

## Velocity and Jacobian #flashcards/jacobian
$S(\omega)$ for $\omega=(\omega_x,\omega_y,\omega_z)$?::$\begin{bmatrix}0&-\omega_z&\omega_y\\\omega_z&0&-\omega_x\\-\omega_y&\omega_x&0\end{bmatrix}$.
$\dot R$ in terms of $\omega$?::$\dot R=S(\omega)R$.
Jacobian column for a revolute joint?::$[z_{i-1}\times(o_n-o_{i-1});\ z_{i-1}]$.
Jacobian column for a prismatic joint?::$[z_{i-1};\ 0]$.
Where do you read $z_{i-1}$ and $o_{i-1}$?::3rd and 4th column of $T^0_{i-1}$.
Singularity condition?::$\det J=0$ (rank drop).
Planar 2R singular configurations?::$\theta_2=0$ or $\pi$ ($\det J_v=a_1a_2\sin\theta_2$).
Consequences of a singularity?::Lost motion directions, infinite joint speeds, unbounded tip forces for bounded torques.
Static force relation?::$\tau=J^TF$.
Pseudo-inverse for a redundant arm?::$J^+=J^T(JJ^T)^{-1}$; null-space term $(I-J^+J)b$.

## Trajectories #flashcards/traj
Path vs trajectory?::Path = geometry only; trajectory = path with time.
Why avoid velocity jumps?::They imply infinite acceleration.
Number of conditions for a cubic / quintic?::4 (q, v at both ends) / 6 (q, v, a at both ends).
Cubic's acceleration property?::Linear in time, discontinuous at the end points.
Quintic with $t_A=0$: first three coefficients?::$c_0=q_A$, $c_1=v_A$, $c_2=a_A/2$.
Size of the remaining system in the quintic shortcut?::$3\times3$ in $c_3,c_4,c_5$.
Exam Q3.2 result?::$c=(0.7854,\ 0,\ 0,\ -1.982,\ 1.611,\ -0.3348)$.
Joint-space vs Cartesian interpolation tip path?::Joint space: curved. Cartesian: straight line.

## Dynamics #flashcards/dyn
Euler–Lagrange equation?::$\frac{d}{dt}\frac{\partial\mathcal L}{\partial\dot q_k}-\frac{\partial\mathcal L}{\partial q_k}=\tau_k$, $\mathcal L=\mathcal K-\mathcal P$.
Standard matrix form?::$D(q)\ddot q+C(q,\dot q)\dot q+g(q)=\tau$.
Kinetic energy in terms of $D$?::$\tfrac12\dot q^TD(q)\dot q$.
Christoffel symbol?::$c_{ijk}=\tfrac12(\partial_{q_i}d_{kj}+\partial_{q_j}d_{ki}-\partial_{q_k}d_{ij})$.
Centrifugal vs Coriolis terms?::$\dot q_i^2$ terms vs $\dot q_i\dot q_j$ ($i\neq j$) terms.
Inertia of a rod about its end?::$ml^2/3$.
Effect of large gear ratio on dynamics?::$\tilde D=D+\text{diag}(J_mr^2)$ becomes diagonal-dominant → decoupled joints.
Potential energy sign?::$\mathcal P=-\sum m_i\mathbf g^Tr_{ci}$.

## Control #flashcards/control
Laplace of $\dot f$?::$sF(s)-f(0)$.
Final value theorem condition?::All poles of $sF(s)$ in the open left half plane.
Poles of a transfer function equal what?::Eigenvalues of $A$ (roots of $D(s)$).
Closed loop $A/(1+AB)$ vs $A/(1-AB)$?::Negative vs positive feedback.
$\zeta$ and $\omega_n$ of $K/(Js^2+Bs+K)$?::$\zeta=B/(2\sqrt{KJ})$, $\omega_n=\sqrt{K/J}$.
Damping regimes?::$\zeta<1$ underdamped, $=1$ critical, $>1$ overdamped.
What does I-action do?::Removes steady-state error for constants (adds a pole at 0), may oscillate.
Why is D never used alone?::Doesn't react to constant error, amplifies noise.
PD steady-state error with disturbance $D$?::$e_{ss}=D/K_p$.
Design gains for desired $\omega,\zeta$?::$K_p=J\omega^2$, $K_d=2\zeta\omega J-B$.
PID stability condition?::$K_i<(B+K_d)K_p/J$.
Reduced DC-motor TF ($L/R\approx0$)?::$\dfrac{K_m/R}{s(Js+B+K_bK_m/R)}$.
Poles of voltage→angle model with $L$?::3 (one at the origin).
Reflected inertia?::$J_L/n^2$.
Steady-state error of P-controlled DC motor under load torque $\tau_l$?::$e_{ss}=R\tau_l/(nK_iK_p)$.
Effect of input saturation?::Slower rise time (and windup with integrators).
Feedforward requirement?::$F$ stable and proper; ideal $F=1/G$.
Why does a larger gear ratio reduce the steady-state error?::Disturbance at the motor $\propto1/r$.

## Sensors and vision #flashcards/vision
Sensor for joint angle / rotation speed / tilt rate?::Encoder / tachometer / gyroscope.
Why fuse sensors?::Noise, limited accuracy/view, failures, cost; combine with uncertainty (Kalman).
Perspective projection?::$u=\lambda x/z$, $v=\lambda y/z$.
Image plane → pixels?::$r=o_r-\alpha_xu$, $c=o_c-\alpha_yv$, $\alpha=1/s$ pix/mm.
Intrinsic vs extrinsic parameters?::Intrinsic: $\lambda,\alpha_x,\alpha_y,o_r,o_c$. Extrinsic: camera pose.
Can one image give a 3D point?::No, depth is lost (need stereo, motion, or known size/depth).
Depth from known size?::$z=\lambda L_{world}/\Delta_{mm}$.
Stereo depth?::$z=\lambda B/(u_1-u_2)$ (disparity).
Eye-in-hand vs eye-to-hand?::Camera on end-effector vs fixed observing the robot.
PBVS vs IBVS?::Error in Cartesian pose vs error in image features.
Thresholding + erosion/dilation do what?::Separate objects from background and clean the binary image.
Why convert $x$ and $y$ separately to mm before Pythagoras?::Pixels are rectangular ($\alpha_x\ne\alpha_y$).
