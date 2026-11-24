---
tags: [robotics, 34753, sensors, vision, camera]
---
# 09 Sensors and Robot Vision
Sources: Lecture 8.1 (sensors), 8.2 handwritten vision notes, Vision exercises (Fall 2025), VisionExam, Exam Q4.1–4.4. Prev: [[08 Control]] · Next: [[10 Project Assignment]]

## 1. Robot sensors 🟦
- **Sensing** = collecting information about the world. A **sensor** maps an environmental attribute to a quantitative measurement via a **transduction principle** (conversion of energy from one form to another, e.g. acoustic → electrical).
- Robots need sensors for: localisation ("where am I?"), obstacle detection ("will I hit anything?"), awareness of surroundings / goal seeking (object area, centre, orientation, distance), and task-specific perception (crop line, fork holes, face tracking).
- Human senses ↔ robot counterparts: vision (cameras), hearing (microphones), touch (force/tactile), smell/taste (chemical sensors).

| Required measurement | Typical sensor |
|---|---|
| Angle of a joint | **Encoder** |
| Speed of rotation | **Tachometer** |
| Absolute position | GPS |
| Distance to an object | optical, ultrasonic proximity, laser distance sensor |
| Touch | switch |
| Quality / size of a part | **machine vision** |
| Direction of travel | compass |
| Rate of tilt | **gyroscope** |
(also: accelerometer, tilt sensors, Hall-effect, IR reflection, gas, etc.)

### Sensor fusion 🟦
One sensor is usually not enough: real sensors are **noisy**, have **limited accuracy**, can **fail** (redundancy), and see only a **limited view**; two cheap sensors can be cheaper than one expensive one.
- **Fusion** = combine data from different sensors, positions and times, taking **uncertainty** into account (**Kalman filtering**, neural networks…).
- Example (IMU): gyroscope → 3D rotation rate; accelerometer → gravity direction/linear acceleration; e-compass → heading; combined → accurate position/orientation.
- **Pipeline:** Sensor 1…N → **pre-processing** each (noise filtering, re-calibration, edge detection, change of representation) → **data fusion**.

## 2. Geometry of image formation 🟦
- An image is a 2D array of **pixels**. **Pinhole camera model**, **perspective projection** through the **centre of projection**.
- Camera frame $\{C\}$: origin at the centre of projection, **$z$ = optical (principal) axis**, $x,y$ in the image-plane directions. The (virtual) **image plane** is at distance $\lambda$ (focal length) along $z$.

### Perspective projection (handwritten notes p.1)
A 3D point $(x,y,z)$ (camera frame) projects to the image-plane coordinates $(u,v)$ by similar triangles:
$$\boxed{u=\lambda\frac xz,\qquad v=\lambda\frac yz}\qquad(\text{units: mm; }\lambda\text{ in mm})$$
(scale factor $k=\lambda/z$ with $k\,[x,y,z]=[u,v,\lambda]$.) Depth information is **lost**: all points along one ray give the same $(u,v)$.

### Image plane → sensor array (pixels) 🟦
Notation: $s_x,s_y$ [mm/pixel] = pixel size; $\alpha_x=1/s_x$, $\alpha_y=1/s_y$ [pixel/mm]; $(r,c)$ = pixel (row, column); $(o_r,o_c)$ = **principal point** (pixel where the optical axis hits the sensor); $\Delta[\text{mm}]=\Delta[\text{pix}]/\alpha$.
$$-\frac u{s_x}=r-o_r,\qquad -\frac v{s_y}=c-o_c\ \Longleftrightarrow\ \boxed{r=o_r-\alpha_xu,\quad c=o_c-\alpha_yv}$$
Combined: $r=o_r-\alpha_x\lambda\dfrac xz$, $\;c=o_c-\alpha_y\lambda\dfrac yz$.

> [!warning] ⚠️ Sign convention: read before the exam
> The *Vision exercises* PDF and the handwritten notes p.3 write the $c$-equation as $v/s_y=c-o_c$ (plus), but p.2 of the notes and **every exam/VisionExam answer** are consistent with the **minus** sign above (🧮 I checked Q4.2, Q4.4 and VisionExam Q4.1 numerically: only the minus form reproduces the answer options).
> **Safest method:** draw the pixel axes ($r$, $c$) next to the camera axes ($x$, $y$) given in the figure and decide the sign from that. In the exam figures $r$ points **left** and $c$ points **up** from the bottom-right corner, while the camera $x$ points right and $y$ down, so both get a minus.

### Forward and backward projection 🟦
- **Forward:** world point $P^w$ → camera frame with the extrinsics → $(u,v)$ → pixels $(r,c)$. *Which pixels does an object at a known world position hit?* Answerable.
- **Backward:** from a 2D pixel to a 3D point is **not possible with a single image** (depth is missing). You need **stereo vision**, **motion**, or **known depth/size** (planar scene).
- **Intrinsic parameters:** $\lambda,\alpha_x,\alpha_y,o_r,o_c$ (camera → pixels). **Extrinsic parameters:** the pose $T_{w}^{c}$ (camera ↔ world). Both are found by **camera calibration** (calibration toolbox).
- World → camera: $p^c=(T^b_c)^{-1}p^b$ and $p^b=T^b_c\,p^c$.

### Worked example (lecturer's handwritten notes) 🟦
Camera on the end-effector: $T^B_c=\begin{bmatrix}1&0&0&0.3\\0&-1&0&0.1\\0&0&-1&0.9\\0&0&0&1\end{bmatrix}$ (camera looks down). Object point in the camera frame $p^c_o=(-0.2,\ 0,\ 0.8)$ m.
With $\lambda/s_x=\lambda/s_y=500$ pix: $u/s_x=500\cdot\frac{-0.2}{0.8}=-125=-(r-o_r)$ → $r-o_r=125$, so with $o_r=320$: $r=445$; $v=0$ → $c=o_c=240$.

## 3. Measuring with a single camera (known depth or size) 🟦
Similar triangles give the **world ↔ image scale**:
$$\frac{\text{world length}}{\text{image length [mm]}}=\frac z\lambda$$
- **Depth from a known size:** $z=\lambda\,\dfrac{L_{world}}{\Delta_{mm}}$. Example: door of width 0.8 m seen as 0.7955 mm → $z=8\cdot0.8/0.7955=8.05$ m.
- **Size from known depth:** $L=\Delta_{mm}\cdot\dfrac z\lambda$.
- Points on a plane parallel to the image plane (constant $z$): $\Delta r\to\Delta X$ via $\Delta X=\dfrac z\lambda\,\dfrac{\Delta r}{\alpha_x}$.
- Pixels are **rectangular** ($\alpha_x\ne\alpha_y$): convert $x$ and $y$ separately before applying Pythagoras.

## 4. Stereo: depth from disparity 🟦
Two parallel cameras separated by baseline $B$ ($^Lt_R=[B,0]^T$). A point has image coordinates $u_1$ (left) and $u_2$ (right). Using $u=\lambda X/Z$ in each camera, with $X_R=X_L-B$:
$$\frac{u_1-u_2}{\lambda}=\frac Bz\ \Longrightarrow\ \boxed{z=\frac{\lambda B}{u_1-u_2}}$$
$u_1-u_2$ is the **disparity**: big disparity = near object. Handwritten example: $B=0.1$ m, $\lambda=1$ m, $P_1=(0.05,0)$, $P_2=(-0.05,0)$ → $z=1$ m.

## 5. Image processing basics 🟦
- **Segmentation by thresholding:** a gray image has 256 gray levels; the **histogram** counts the occurrences of each level; choose a threshold that partitions the image into **objects and background** (homogeneous regions).
- Clean-up with **erosion** and **dilation** (morphological operations) 📖: erosion shrinks objects/removes small specks; dilation grows objects/fills holes.
- Image **features** (corners, edges, centroids, area, orientation) feed the control loop; **object area, centre, number, orientation** are typical outputs.

## 6. Visual servoing 🟦
Use visual information **in the feedback loop** (better accuracy than "look and move"). Combines image processing, kinematics/dynamics, control theory and real-time computing.
- **Camera configurations:** **eye-in-hand** (camera on the end-effector), **eye-to-hand** (fixed camera watching the robot), or cooperation of both.
- **PBVS (position-based):** image → feature extraction → **pose estimation** ($^cx$ estimated) → compare with desired pose $^cx_d$ → **Cartesian control law** → robot. The error is in Cartesian space.
- **IBVS (image-based):** image → feature extraction → compare features $f$ with desired features $f_d$ → **feature-space control law** → robot. No explicit pose estimation.
- Related courses: 34754 Autonomous Robot Systems, 34757 Unmanned Autonomous Systems.

## 7. Solved exercises (Fall 2025 Vision exercises) 🟦
Camera: $\lambda=8$ mm, $\alpha_x\approx79.2$–$79.3$ pix/mm, $\alpha_y=120.5$ pix/mm, $o_r=o_c=250$ (the VisionExam also uses 250/250; the **MCQ exam Q4.1–4.4 uses $o_r=250,\ o_c=160$**, i.e. a 500×320 sensor).

**Ex. 1: Drone camera.**
1. $\Delta$ in pixels between $P_1=[476,380]$ and $P_2=[147,117]$: $\sqrt{329^2+263^2}=421$ pix.
2. In mm: $\sqrt{(329/\alpha_x)^2+(263/\alpha_y)^2}=4.69$ mm (convert each axis separately).
3. Car at 43 km/h = 11.94 m/s for $\Delta t=3$ s → **35.83 m**.
4. Scale = $35.83/4.69=7.636$ m/mm ⇒ altitude $Z=\lambda\cdot7.636=61.09$ m. With the base point at pixel $[450,50]$: camera offset $\approx(19.28,\,-12.67,\,61.09)$ m (x: $(450-250)/\alpha_x\cdot7.636$; y: $(50-250)/\alpha_y\cdot7.636$).

**Ex. 2: Couch and door** (wall 8 m away, door 0.8 m wide, image plane parallel to the wall).
- Door in image: $|P_{9x}-P_{8x}|=63$ pix = 0.7955 mm → wall depth $z=8\cdot0.8/0.7955=8.046$ m.
- Length = $|P_{6x}-P_{5x}|=158$ pix = 1.995 mm → $\times0.8/0.7955$ = **2.00 m**.
- Height = $|P_{6y}-P_{7y}|=109$ pix = 0.905 mm → $\times z/\lambda$ = **0.91 m**.
- Depth of the front face from the edge $P_2P_3$ (123 pix = 1.021 mm, same 0.91 m height): $z_{P_2}=0.910\cdot8/1.021=7.13$ m ⇒ **width = 8.046 − 7.130 = 0.92 m** 🧮 (the printed solution stops at $z_{P_2}$).
- **Can the couch pass the 0.8 m door? No** (smallest couch dimension 0.91 m > 0.8 m).

**Ex. 3: Heights of people.**
1. Which is taller from the image alone? **Cannot be said** (depth unknown).
2. $z_{P1}=5$ m: $\Delta y=331$ pix = 2.747 mm → $2.747\cdot5/8=1.72$ m. $z_{P2}=8$ m: $211$ pix = 1.751 mm → $1.751\cdot8/8=1.75$ m. **P2 is taller** (1.75 m vs 1.72 m).

## 8. Self-check
- Why can one image not give 3D position? → projection loses depth $z$.
- Which quantity must be known to turn pixel distances into metres? → depth (or an object of known size).
- Eye-in-hand vs eye-to-hand? → camera on the end-effector vs fixed in the workspace.
- PBVS vs IBVS? → error in Cartesian pose vs error in image features.
