
<p align="center">
  <img width="700" height="90" alt="image" src="https://github.com/user-attachments/assets/a9600b74-0941-4c72-833f-af076c559691" />
</p>

<h1 align="center">Introduction to Data-Driven Control of Drones - 22AIE448</h1>

<h1 align="center">Autonomous Landing of Quadrotor UAV Using AprilTags Vision Positioning and Quaternion-based Attitude Representation </h1>

### *Team Members*
Police Aryan - CB.SC.U4AIE24241\
Gayathri Vaishnav - CB.SC.U4AIE24337\
Rohit Vardhan  - CB.SC.U4AIE24231\
Sai Reddy - CB.SC.U4AIE24205\
Sairi Manvik - CB.SC.U4AIE24352

## 1. Abstract

Autonomous landing remains one of the most challenging phases of quadrotor UAV flight, particularly in GPS-denied environments such as indoor spaces, dense urban canyons, and forested or covered terrain, where satellite navigation becomes unreliable or entirely unavailable. This project presents an onboard, vision-based autonomous landing and dynamic tracking system for a quadrotor UAV that relies solely on a downward-facing monocular camera and multi-scale AprilTag fiducial markers for real-time relative pose estimation. Building on the foundational architecture proposed by Li et al. (2019), the system employs two coplanar AprilTags of different sizes to maintain accurate positioning across the full descent  from high altitude to touchdown  and a classical PID controller to convert position error into stable velocity commands.

The central contribution of this work is the replacement of the conventional rotation-matrix-based attitude representation with a **Unit Quaternion and Sandwich Operator** formulation, directly applying the quaternion theory covered in course 22AIE448. This substitution eliminates the risk of gimbal lock inherent to Euler-angle-based representations, provides a continuous and numerically stable method for transforming position vectors between the camera and body frames, and improves control smoothness during aggressive banking or turning maneuvers while tracking a moving platform. The resulting system is designed to achieve sub-1% landing error and robust tracking performance on both static and moving landing platforms, validating the practical benefit of quaternion-based attitude representation in a real-world robotics application.

**Keywords:** UAV autonomous landing, AprilTag, visual positioning, unit quaternion, Sandwich Operator, PID control, GPS-denied 


## 2. Introduction

Autonomous landing is widely regarded as one of the most safety-critical and technically demanding phases of unmanned aerial vehicle (UAV) operation, since even small errors in position or orientation during the final descent can result in a failed landing, damage to the vehicle, or harm to nearby people and property. This precision is especially vital for emerging last-mile logistics applications, such as the rapid delivery of critical medications, blood units, and emergency supplies directly to residential doorsteps or confined home landing pads where safety margins are exceptionally narrow. While GPS-based navigation performs reliably in open outdoor environments, its accuracy degrades sharply or fails outright in scenarios common to real-world deployment—such as flight between tall buildings where signals reflect and multipath errors accumulate, indoor operation where satellite visibility is entirely blocked, movement beneath dense tree canopies, and other GPS-denied settings encountered in residential neighborhoods, warehouses, disaster zones, or dense urban canyons. In these conditions, a quadrotor cannot depend on satellite positioning to execute a safe, precise delivery and touchdown, creating a clear and practical need for an alternative, self-contained sensing approach that does not rely on external infrastructure.

Vision-based positioning offers a practical and low-cost solution: by mounting a downward-facing monocular camera on the UAV and placing known fiducial markers—specifically AprilTags—on the target landing platform, the drone computes its relative position and orientation with respect to the pad in real time, entirely onboard, using only the visual geometry of the tag. This approach removes dependency on GPS altogether while remaining computationally lightweight enough to run on modest onboard hardware, making it well-suited to small and medium-sized delivery quadrotors with limited payload and processing capacity.

Building on this framework, the present project takes as its architectural foundation the method proposed by Li et al. in *"UAV Autonomous Landing Technology Based on AprilTags Vision Positioning Algorithm"* (2019), which demonstrated a complete onboard pipeline for AprilTag-based autonomous landing using a dual-tag detection strategy—a large tag for reliable acquisition at high altitude and a nested small tag for fine positioning close to the ground—combined with a classical PID velocity controller that converts estimated position errors into smooth, stable motor commands. This project adopts that baseline pipeline, reusing the camera-based detection stage, the dual-tag altitude-dependent switching logic, and the core PID control structure, while extending the attitude representation stage using unit quaternions and the Sandwich Operator to eliminate gimbal lock and improve rotation efficiency.

### 2.1 Our Contribution

The base paper represents drone orientation using a 3×3 **rotation matrix**, derived from sequential Euler-angle rotations. While functionally adequate under normal flight conditions, this representation is known to suffer from **gimbal lock**  a singularity that can cause a loss of one rotational degree of freedom during large-angle maneuvers and is computationally redundant compared to more compact alternatives.

In this project, we address this limitation by introducing **Unit Quaternions** and the **Sandwich Operator** to represent and transform orientation, replacing the rotation-matrix stage of the original pipeline. This is a direct, applied extension of the quaternion theory taught in course *22AIE448 – Introduction to Drones*, and forms the core technical contribution of this work. Specifically:

- The rotation matrix obtained from AprilTag pose estimation is converted into a unit quaternion, avoiding the singularities associated with Euler-angle representations.
- The Sandwich Operator is used to transform the tag's position vector from the camera frame into the drone's body frame in a numerically stable, continuous manner.
- This improves control smoothness and robustness, particularly during banking or turning maneuvers while tracking a moving landing platform.

By combining the proven vision-based landing architecture of the base paper with a quaternion-based attitude formulation, this project demonstrates a practical, singularity-free improvement to an established autonomous landing method.

### 3.  Base paper/ Literature Survey 
<h2 align="center">Table 3.1</h2>

| # | Reference | Authors / Year | Title    | Key Methodology | Why Used in Our Project | Limitations |
|---|---|---|---|---|---|---|
| 1 | Base Paper | Z. Li, Y. Chen, H. Lu, H. Wu, L. Cheng (2019) | "UAV Autonomous Landing Technology Based on AprilTags Vision Positioning Algorithm," 38th Chinese Control Conference (CCC) | Monocular camera + dual-sized AprilTags for pose estimation; classical PID controller for velocity commands; tested on static and moving platforms | This is our **primary base paper** we reuse its full pipeline (dual-tag detection, pose estimation, PID control) and extend the attitude representation stage with quaternions | Uses rotation matrices (Euler-based) for attitude susceptible to gimbal lock; fixed PID gains; single test environment, no robustness analysis |
| 2 | Fiducial Marker | E. Olson (2011) | "AprilTag: A Robust and Flexible Visual Fiducial System," IEEE ICRA | Introduces the AprilTag fiducial marker family 2D barcode-like tags with robust detection, sub-pixel corner localization, and low false-positive rate even under occlusion/lighting variation | Foundational reference for the **marker system itself** explains why AprilTag was chosen over other fiducials (accuracy, open-source, low computational cost) for our detection stage | Detection accuracy still degrades at extreme viewing angles or very low resolution; no built-in multi-scale/multi-tag switching strategy (added later by other works) |
| 3 | Attitude Representation | E. Fresk, G. Nikolakopoulos (2013) | "Full Quaternion Based Attitude Control for a Quadrotor," European Control Conference (ECC) | Formulates full nonlinear quadrotor attitude control directly in unit-quaternion space, avoiding Euler-angle linearization and singularities | Directly supports our **core contribution** justifies replacing the base paper's rotation-matrix attitude representation with unit quaternions + Sandwich Operator for singularity-free, smoother control | Focuses on attitude control law design, not vision-based pose estimation; doesn't address marker detection or landing-specific error dynamics |
| 4 | Moving-Platform Landing | D. Falanga, A. Zanchettin, A. Simovic, J. Delmerico, D. Scaramuzza (2017) | "Vision-Based Autonomous Quadrotor Landing on a Moving Platform," IEEE SSRR | Onboard vision pipeline with marker detection, relative velocity estimation, and adaptive control law for landing on platforms moving at varying speeds | Supports our **dynamic/moving-platform validation** goal provides additional benchmarking methodology and control strategy references beyond the base paper's simpler PID approach | Requires higher onboard computational power for real-time relative velocity estimation; assumes platform motion is roughly planar and bounded in speed |

### 3.2  Base paper math:

## Base Paper Equations (Li et al., 2019 — No Quaternions)

### 1. Pixel Conversion (Camera → Digital Image)

$$
u = \frac{x}{dx} + u_0 \qquad v = \frac{y}{dy} + v_0
$$

**In plain words:** A camera sensor sees the world as a continuous picture, but a computer stores it as a grid of pixels (like graph paper). This equation just says: "take the real position on the sensor and tell me which pixel box it lands in."

| Variable | What it is | Why it's needed |
|---|---|---|
| `x, y` | Where a point actually is on the camera sensor (in millimeters) | This is the "real" physical location before we turn it into pixels |
| `dx, dy` | How big one pixel is (in millimeters) | Tells us how many pixels fit into one millimeter — this does the mm → pixel conversion |
| `u, v` | The pixel address of that point (column, row) | This is what the computer actually works with — an image is just numbers in a pixel grid |
| `u0, v0` | The pixel at the exact center of the image | Cameras don't always have their center at pixel (0,0), so we shift by this amount to correct for that |

---

## After this they use a library called pupil_apriltags that identifies the april tag and then continues the future steps 

### 2. One-Axis Rotation (Roll)

$$
R_x(\phi) =
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi & \cos\phi
\end{bmatrix}
$$

**In plain words:** Imagine tilting a tray left-to-right — that's roll. This matrix is a "tilt calculator": give it an angle, and it tells you how every point on the tray moves.

| Variable | What it is | Why it's needed |
|---|---|---|
| `φ` (phi) | The roll angle — how much the drone/tag is tilted sideways | Without this, we can't tell how the object rotated around one axis |
| `R_x(φ)` | The 3×3 matrix itself | A reusable "rotation machine" — multiply it with a point and it spins that point by angle φ |

---

### 3. Full 3D Rotation (Combining Roll, Pitch, Yaw)

$$
R = R_z(\psi)\, R_y(\theta)\, R_x(\phi)
$$

**In plain words:** A drone doesn't just tilt in one direction — it rolls, pitches (nose up/down), and yaws (spins left/right) all at once. Chaining three simple tilt calculators together gives the *full* rotation.

| Variable | What it is | Why it's needed |
|---|---|---|
| `ψ` (psi) | Yaw angle — spinning left/right | Captures rotation around the vertical axis |
| `θ` (theta) | Pitch angle — nose up/down | Captures forward/backward tilt |
| `φ` (phi) | Roll angle — tilting side to side | Captures sideways tilt |
| `R` | The combined 3×3 rotation matrix | **This is the base paper's final answer for orientation** — no quaternion involved |

---

### 4. PID Controller (Turning Error into Speed Commands)

$$
V = K_p \cdot e \;+\; K_i \int e\, dt \;+\; K_d \cdot (e - e_{prev})
$$

**In plain words:** "How fast should the drone move to fix its position error?" It looks at three things — how far off you are *right now*, how far off you've *been* over time, and how *quickly* the error is changing — and blends all three into one speed command.

| Variable | What it is | Why it's needed |
|---|---|---|
| `e` | The current position error (how far the drone is from where it should be) | The main signal — no error means no correction needed |
| `Kp · e` | Proportional term — reacts to the error right now | Bigger error → bigger push toward the target |
| `Ki ∫e dt` | Integral term — adds up all past error over time | Corrects small, persistent errors (like drift) that proportional alone can't fix |
| `Kd(e − e_prev)` | Derivative term — reacts to how fast the error is changing | Acts like a brake — stops overshoot and oscillation |
| `Kp, Ki, Kd` | Tuning knobs (gains) for each term | Control how aggressive or gentle each correction is |
| `V` | The final speed command sent to the drone | The actual output that moves the drone |

---

### 5. Position Error

$$
\text{Error} = \text{Desired Position} - \text{Current Position}
$$

**In plain words:** "Where you want to be" minus "where you actually are." That gap is what the PID controller above is trying to shrink to zero.

| Variable | What it is | Why it's needed |
|---|---|---|
| Desired Position | Where the drone should be (usually right above the tag) | The target — set by the landing task |
| Current Position | Where the drone actually is, from AprilTag pose estimation | Our best real-time estimate of the truth |
| Error | The gap between the two | Drives the entire PID control loop |

---

## Our Contribution (Not in the Base Paper)

The base paper stops at equation 3 — a plain rotation matrix. **Everything below is our own addition**, applying quaternion theory from course 22AIE448.

## A. Turning the Rotation Matrix into a Quaternion

Given the Euler angles:

$$
\phi = \text{Roll}, \qquad \theta = \text{Pitch}, \qquad \psi = \text{Yaw}
$$

Define the sine and cosine of the half-angles:

$$
c_x = \cos\left(\frac{\phi}{2}\right), \qquad s_x = \sin\left(\frac{\phi}{2}\right)
$$

$$
c_y = \cos\left(\frac{\theta}{2}\right), \qquad s_y = \sin\left(\frac{\theta}{2}\right)
$$

$$
c_z = \cos\left(\frac{\psi}{2}\right), \qquad s_z = \sin\left(\frac{\psi}{2}\right)
$$

The quaternion is calculated as:

$$
q = w + xi + yj + zk
$$

where:

$$
w = c_x c_y c_z + s_x s_y s_z
$$

$$
x = s_x c_y c_z - c_x s_y s_z
$$

$$
y = c_x s_y c_z + s_x c_y s_z
$$

$$
z = c_x c_y s_z - s_x s_y c_z
$$

Therefore, the quaternion is:

**q = [w, x, y, z]ᵀ**

where:

- w = cₓ·c_y·c_z + sₓ·s_y·s_z
- x = sₓ·c_y·c_z − cₓ·s_y·s_z
- y = cₓ·s_y·c_z + sₓ·c_y·s_z
- z = cₓ·c_y·s_z − sₓ·s_y·c_z

| Variable | What it is | Why it's needed |
|---|---|---|
| $r_{ij}$ | The element in row $i$, column $j$ of the rotation matrix $R$ | The raw ingredients — read directly from the rotation matrix |
| $q_0 \ (= w)$ | The "how much rotation" part of the quaternion (the scalar/real part) | Tells you the *size* (angle) of the rotation |
| $q_1, q_2, q_3 \ (= x, y, z)$ | The "which direction" part of the quaternion (a 3D vector) | Together with $q_0$, fully describes the same rotation — just more compactly than a 3×3 matrix |
### B. Rotating a Vector with a Quaternion (Sandwich Operator)

$$
L_q(v) = q\, v\, q^{*}
$$

**In plain words:** Think of `v` as a small arrow (like "the tag is 2 meters that way"). To rotate that arrow using a quaternion, you "sandwich" it — multiply the quaternion on one side and its conjugate on the other. What comes out is the same arrow, just rotated.



### Coordinate Frame Transformation & Sandwich Operator

The onboard camera measures position error in its own optical frame, but the flight controller requires this error expressed in the drone's **body frame** ($B$). As the quadrotor tilts and pitches during flight, these two coordinate frames no longer align.



Camera Frame (C)  ───[ Sandwich Operator: q v q* ]───►  Body Frame (B)  ───►  PID Controller
(Raw Error Vector)                                     (Aligned Error)       (Actuator Commands)



#### Why the Sandwich Operator is Required

* **Quaternion Representation:** A unit quaternion $\mathbf{q}$ stores the 3D orientation/tilt of the vehicle, but a quaternion alone cannot rotate a 3D vector without an algebraic operator.
* **Double-Cover & Distortion Elimination:** Simple left-multiplication ($\mathbf{q}\mathbf{v}$) produces a four-dimensional result with unwanted scaling/projection artifacts. The **Sandwich Operator** maps a pure spatial vector back into the 3-vector space cleanly:

$$v' = q \, v \, q^{*}$$

Where:
* $v = [0, \mathbf{e}_{cam}]^T$ is the raw visual position error treated as a pure vector quaternion.
* $q$ is the unit attitude quaternion representing current vehicle tilt.
* $q^{*} = q^*$ is the quaternion conjugate (inverse for unit quaternions).
* $v' = [0, \mathbf{e}_{body}]^T$ is the transformed position error vector.

#### Integration in Control Loop

The resulting vector $v'$ represents the exact relative position error re-expressed in the drone's body frame:

$$\mathbf{e}_{body} = \text{vec}\left( q \, [0, \mathbf{e}_{cam}] \, q^{*} \right)$$

This transformed error is routed directly into the **PID controller** to generate accurate thrust, roll, and pitch commands without frame misalignment errors.


| Variable | What it is | Why it's needed |
|---|---|---|
| `q` | The quaternion describing the rotation (from step A) | The "rotation instruction" |
| `q*` | The conjugate of `q` — `q` flipped/reversed | Needed to correctly complete the rotation and leave a clean result |
| `v` | The vector being rotated — here, the tag's position | We want the tag's position *relative to the drone*, not the camera |
| `L_q(v)` | The final, rotated vector | The tag's position now expressed in the drone's own frame of reference |


### 3.3  Base paper link:
[IEEE Xplore](https://ieeexplore.ieee.org/document/8865757) 

### 4.  Complete System Pipeline

<h2 align="center">Fig 4.1</h2>

<p align="center">
<img width="700" height="800" alt="image" src="https://github.com/user-attachments/assets/8f361a3d-0164-447e-81aa-852b44e49711" />
</p>

### 5.Methodology

## Methodology

Our implementation is a full physics-based simulation of the base paper's pipeline, built in MuJoCo, with three main modules working in a closed loop at each control step:

1. **Physics simulation (`envs/quad_env.py`)** — A high-fidelity MuJoCo model of a DJI M100-class quadrotor (3.5 kg) is stepped at 500 Hz internally (`DT_SIM = 0.002s`) and controlled at 50 Hz (`DT_CTRL = 0.02s`). It includes motor lag, ground-effect lift, and Dryden wind turbulence. The drone's true orientation is read directly from MuJoCo as a quaternion `[qw, qx, qy, qz]` and converted to Euler angles for the vision model.

2. **Vision simulation (`vision/camera_sim.py`, `vision/apriltag_detect.py`)** — A synthetic downward camera (640×480, FX=FY=650) renders the ground, landing pad, and a perspective-warped AprilTag (`tag36h11`) into an image, with Gaussian blur and pixel noise added for realism. The tag physically switches between a large (0.80 m) and small (0.20 m) marker at a 1.8 m altitude threshold. On the detection side, `pupil_apriltags` locates the tag in the image, and OpenCV's `solvePnP` recovers the tag's pose relative to the camera. The world-frame altitude and lateral offsets are then extracted using the world→body and body→camera rotation matrices, and smoothed with an exponential moving average (EMA) filter.

3. **Control (`control/pid_controller.py`)** — A finite-state controller (`SEARCH → APPROACH → DESCEND_HIGH → DESCEND_LOW → LAND`) selects a target descent speed for the current phase and feeds the lateral (X, Y) position errors into two independent PID loops. Vertical velocity is tracked with a proportional velocity loop. The resulting lateral/vertical accelerations are converted into thrust forces (with acceleration and thrust clipping for realism) and applied back into the MuJoCo physics step, closing the loop.

This differs from the base paper in two important ways: pose estimation uses OpenCV's iterative PnP solver instead of a closed-form rotation-matrix decomposition, and the controller is phase-aware (it changes behaviour by altitude/lock state) rather than a single flat PID loop — closer to how a real onboard autopilot would behave.



### 6.Methodology Block Diagram

<h2 align="center">Fig 6.1</h2>
<p align="center">
<img width="700" height="700" alt="image" src="https://github.com/user-attachments/assets/6e49ec3c-b354-4c10-9adc-d0728c1e1b22" />
</p>

### 7.Results

<img width="1919" height="1079" alt="Screenshot 2026-08-20 172216" src="https://github.com/user-attachments/assets/5578fd25-4eea-4955-9043-874da225b3f1" />
<img width="1934" height="742" alt="trajectory_summary" src="https://github.com/user-attachments/assets/246ad55c-c675-4e80-b1f0-66becbccce63" />
<img width="1934" height="1183" alt="pid_comparison_plots" src="https://github.com/user-attachments/assets/a8fc12bc-8819-4eb7-9a15-b71622edbe16" />
<img width="1771" height="1036" alt="dynamic_tracking_plots" src="https://github.com/user-attachments/assets/6d8aaddf-aad3-4c52-b431-5fa158a2c749" />




### 8.Conclusion

<h3 align="center">Table 8.1</h3>

| # | Reference | Link |
|---|---|---|
| 1 | Z. Li, Y. Chen, H. Lu, H. Wu, L. Cheng, "UAV Autonomous Landing Technology Based on AprilTags Vision Positioning Algorithm," CCC 2019 | [IEEE Xplore](https://ieeexplore.ieee.org/document/8865757) |
| 2 | E. Olson, "AprilTag: A Robust and Flexible Visual Fiducial System," ICRA 2011 | [IEEE Xplore](https://ieeexplore.ieee.org/document/5979561) · [Free PDF (UMich)](https://april.eecs.umich.edu/pdfs/olson2011a.pdf) |
| 3 | E. Fresk, G. Nikolakopoulos, "Full Quaternion Based Attitude Control for a Quadrotor," ECC 2013 | [Google Scholar](https://scholar.google.com/scholar?q=Full+Quaternion+Based+Attitude+Control+for+a+Quadrotor+Fresk+Nikolakopoulos) |
| 4 | D. Falanga, A. Zanchettin, A. Simovic, J. Delmerico, D. Scaramuzza, "Vision-Based Autonomous Quadrotor Landing on a Moving Platform," SSRR 2017 | [IEEE Xplore](https://ieeexplore.ieee.org/document/8088164) · [Free PDF (UZH RPG)](http://rpg.ifi.uzh.ch/docs/SSRR17_Falanga.pdf) |


