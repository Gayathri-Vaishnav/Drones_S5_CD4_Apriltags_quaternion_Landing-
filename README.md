

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

Autonomous landing remains one of the most challenging phases of quadrotor UAV flight, particularly in GPS-denied environments such as indoor spaces, dense urban canyons, and forested or covered terrain, where satellite navigation becomes unreliable or entirely unavailable. This project presents an onboard, vision-based autonomous landing and dynamic tracking system for a quadrotor UAV that relies on a downward-facing monocular camera and dual-scale AprilTag fiducial markers for real-time relative pose estimation. Building on the architecture proposed by Li et al. (2019), every landing pad carries two AprilTags of different sizes: a 0.42 m tag readable up to 8.73 m and a 0.12 m tag readable below 2.49 m. Together they keep the pad in view through the whole descent, from search altitude to touchdown. We extend the base paper with an **addressing layer**: ten numbered landing pads, four on the street, three on moving vehicles and three on building roofs. The drone flies to the chosen address, tracks the pad while it moves, and lands on it.

The central contribution of this work is the replacement of the base paper's Euler-angle rotation matrices with **Unit Quaternions and the Sandwich Operator**, directly applying the quaternion theory covered in course 22AIE448. Quaternions are used throughout the system:

- the camera → body → world frame transformations
- the full attitude controller, where the attitude error is $q^*\otimes q_{des}$ with a shortest-path sign check
- SLERP smoothing of the pad heading

This removes gimbal lock and the ±180° heading wrap-around. In addition, we upgrade the base paper's fixed-gain PID with integral anti-windup, pad-velocity feed-forward and altitude gain scheduling. We also add an α-β-γ tracker with a coordinated-turn model for moving pads, and a geometric decision height below which the drone commits to landing.

The same model was implemented on three independent platforms: MuJoCo (Python, with rendered camera images and a real AprilTag detector), MATLAB and NetLogo. It landed **120 out of 120** test missions, with mean touchdown errors of **2.41 cm, 3.38 cm and 2.80 cm** respectively on a 1.5 m pad. With the base paper's PID alone, the drone landed on **0 of 6** moving pads; the full controller landed on **6 of 6**.

**Keywords:** UAV autonomous landing, AprilTag, visual positioning, unit quaternion, Sandwich Operator, PID control, feed-forward, moving-target tracking, GPS-denied


## 2. Introduction

Autonomous landing is widely regarded as one of the most safety-critical and technically demanding phases of unmanned aerial vehicle (UAV) operation. Even small errors in position or orientation during the final descent can result in a failed landing, damage to the vehicle, or harm to nearby people and property. This precision is especially vital for emerging last-mile logistics applications, such as the rapid delivery of critical medications, blood units and emergency supplies directly to residential doorsteps, rooftop pads or delivery vehicles, where safety margins are exceptionally narrow.

GPS-based navigation performs reliably in open outdoor environments, but its accuracy degrades sharply or fails outright in scenarios common to real-world deployment:

- flight between tall buildings, where signals reflect and multipath errors accumulate
- indoor operation, where satellite visibility is entirely blocked
- movement beneath dense tree canopies
- other GPS-denied settings, such as residential neighborhoods, warehouses and disaster zones

Even a radio beacon (such as an AirTag) only brings the drone to within a few metres of the target, which is larger than the landing pad itself. This creates a clear and practical need for an alternative, self-contained sensing approach for the final approach.

Vision-based positioning offers a practical and low-cost solution. A downward-facing monocular camera is mounted on the UAV and known fiducial markers, specifically AprilTags, are placed on the target landing platform. The drone then computes its position and orientation relative to the pad in real time, entirely onboard, using only the visual geometry of the tag. This approach is computationally lightweight enough to run on modest onboard hardware, which suits small and medium-sized delivery quadrotors with limited payload and processing capacity.

The project takes as its architectural foundation the method proposed by Li et al. in *"UAV Autonomous Landing Technology Based on AprilTags Vision Positioning Algorithm"* (2019). That paper demonstrated a complete onboard pipeline for AprilTag-based autonomous landing, combining two parts:

- a dual-tag detection strategy: a large tag for reliable acquisition at high altitude and a small tag for fine positioning close to the ground
- a classical PID velocity controller that converts estimated position errors into smooth, stable velocity commands

This project adopts that baseline pipeline, including the camera-based detection stage, the dual-tag switching logic and the core PID structure with the paper's own gains. It extends the pipeline in three directions:

1. A unit-quaternion attitude representation with the Sandwich Operator.
2. Guidance that can land on moving and turning platforms.
3. Addressed delivery to ten different pads on the ground, on vehicles and on rooftops.

### 2.1 Our Contribution

The base paper represents drone orientation using a 3×3 **rotation matrix**, built from sequential Euler-angle rotations (its Eq. 6–8). This works under normal flight conditions, but the representation suffers from **gimbal lock**: a singularity that loses one rotational degree of freedom at ±90° pitch. Angle differences also **wrap around** at ±180°, so a heading change from 179° to −179° is computed as 358° instead of 2°. A pad on a vehicle driving in circles crosses this line on every lap.

In this project, we address these limitations by introducing **Unit Quaternions** and the **Sandwich Operator** to represent and transform orientation, replacing the rotation-matrix stage of the original pipeline. This is a direct, applied extension of the quaternion theory taught in course *22AIE448 – Introduction to Drones*, and it is the core technical contribution of this work. Specifically:

- The drone's attitude is a unit quaternion $q$. The tag's position vector is moved from the camera frame to the body and world frames with the Sandwich Operator $q\,v\,q^*$, so no Euler angle is computed anywhere in the control loop.
- The attitude controller builds the desired attitude $q_{des}$ directly from the required thrust vector. The attitude error is a single product, $q_{err}=q^*\otimes q_{des}$, and one sign check (flip if $w<0$) always takes the shortest rotation. The base paper hands this step to the DJI autopilot.
- The pad's heading is smoothed with quaternion **SLERP**, which cut heading noise from 2.7° to 1.6° and pad-centre noise from 1.9 cm to 1.1 cm.
- This improves control smoothness and robustness, particularly during banking or turning maneuvers while tracking a moving landing platform.

Further extensions beyond the base paper:

- **Addressed pads:** 10 pads, each carrying its own tag IDs (k and k+10). A radio fix (±2.5 m) brings the drone close; the camera then accepts only the addressed tag.
- **Dual-scale tag geometry:** the read limits $h_{max}=f\,s/20$ give 8.73 m for the large tag and 2.49 m for the small tag, with a measured hand-over window of 1.06–2.49 m. A **decision height** of $h_{blind}=0.25+1.73e$ m tells the drone when to commit and land instead of climbing back up.
- **Upgraded PID guidance:** the paper's gains (0.20, 0.03, 0.35) are kept, with four additions: a filtered derivative, integral anti-windup (leak), pad-velocity feed-forward and altitude gain scheduling. Moving-pad landings went from 0/6 to 6/6.
- **Moving-pad tracker:** an α-β-γ filter with a coordinated-turn model $\Omega=(v\times a)_z/|v|^2$ predicts where a turning vehicle will be.
- **Three platforms:** the same model was built in MuJoCo, MATLAB and NetLogo, with 40 missions each and 120/120 landings in total.

By combining the proven vision-based landing architecture of the base paper with a quaternion-based attitude formulation, this project demonstrates a practical, singularity-free improvement to an established autonomous landing method.

<h2 align="center">Fig 2.1</h2>

<p align="center">
  <img width="1000" alt="Fig 2.1" src="images/fig2_1_process_overview.png" />
</p>

<p align="center"><i>How one delivery works. (a) The radio beacon brings the drone within ±2.5 m of the address; the first decode of tag ID k switches guidance to vision; the drone descends inside the cone, hands over from the large tag to the small tag between 2.49 m and 1.06 m, and commits below the decision height. (b)–(d) What the onboard camera sees at 8 m, 2 m and 0.45 m (computed from the pinhole model). (e) Layout of one pad.</i></p>


### 3.  Base paper/ Literature Survey 
<h2 align="center">Table 3.1</h2>

| # | Reference | Authors / Year | Title    | Key Methodology | Why Used in Our Project | Limitations |
|---|---|---|---|---|---|---|
| 1 | Base Paper | Z. Li, Y. Chen, H. Lu, H. Wu, L. Cheng (2019) | "UAV Autonomous Landing Technology Based on AprilTags Vision Positioning Algorithm," 38th Chinese Control Conference (CCC) | Monocular camera + dual-sized AprilTags for pose estimation; classical PID controller for velocity commands; tested on static and moving platforms | This is our **primary base paper**. We reuse its full pipeline (dual-tag detection, pose estimation, PID control with its published gains) and extend the attitude representation stage with quaternions | Uses rotation matrices (Euler-based) for attitude, which are susceptible to gimbal lock; fixed PID gains that lag a moving pad; single test environment, no robustness analysis; typos in its Eq. (5), (6), (8) |
| 2 | Fiducial Marker | E. Olson (2011) | "AprilTag: A Robust and Flexible Visual Fiducial System," IEEE ICRA | Introduces the AprilTag fiducial marker family: 2D barcode-like tags with robust detection, sub-pixel corner localization, and a low false-positive rate even under occlusion and lighting variation | Foundational reference for the **marker system itself**. It explains why AprilTag (tag36h11) was chosen over other fiducials (accuracy, open source, low computational cost) for our detection stage | Detection accuracy still degrades at extreme viewing angles or very low resolution; no built-in multi-scale/multi-tag switching strategy (added later by other works) |
| 3 | Attitude Representation | E. Fresk, G. Nikolakopoulos (2013) | "Full Quaternion Based Attitude Control for a Quadrotor," European Control Conference (ECC) | Formulates full nonlinear quadrotor attitude control directly in unit-quaternion space, avoiding Euler-angle linearization and singularities | Directly supports our **core contribution**. It justifies replacing the base paper's rotation-matrix attitude representation with unit quaternions + the Sandwich Operator for singularity-free, smoother control | Focuses on attitude control law design, not vision-based pose estimation; doesn't address marker detection or landing-specific error dynamics |
| 4 | Moving-Platform Landing | D. Falanga, A. Zanchettin, A. Simovic, J. Delmerico, D. Scaramuzza (2017) | "Vision-Based Autonomous Quadrotor Landing on a Moving Platform," IEEE SSRR | Onboard vision pipeline with marker detection, relative velocity estimation, and an adaptive control law for landing on platforms moving at varying speeds | Supports our **dynamic/moving-platform validation** goal. It motivated our pad-velocity feed-forward and tracker beyond the base paper's simpler PID approach | Requires more onboard computation for real-time relative velocity estimation; assumes platform motion is roughly planar and bounded in speed |
| 5 | Geometric Attitude Control | T. Lee, M. Leok, N. H. McClamroch (2010) | "Geometric Tracking Control of a Quadrotor UAV on SE(3)," IEEE CDC | Builds the desired attitude from the required thrust vector ($b_3=f/\lvert f\rvert$, heading by cross products) and controls attitude without Euler angles | Our attitude loop uses this construction: thrust vector → $R_{des}$ → $q_{des}$ → quaternion error → torque | Assumes an accurate model and full-state feedback; no vision or landing logic |
| 6 | Quaternion Interpolation | K. Shoemake (1985) | "Animating Rotation with Quaternion Curves," SIGGRAPH | Introduces SLERP, constant-speed interpolation along the shortest arc of the unit-quaternion sphere | Used to smooth the measured pad heading, which feeds the 0.40 m tag-to-centre offset | Interpolation only; says nothing about estimation noise or filtering gains |
| 7 | Target Tracking | Y. Bar-Shalom, X. R. Li, T. Kirubarajan (2001) | *Estimation with Applications to Tracking and Navigation*, Wiley | α-β-γ filters, the coordinated-turn model and Interacting Multiple Model (IMM) estimation | Basis of our moving-pad tracker and turn-rate estimate; IMM is our proposed future work | A single-model filter must switch between straight and turning motion |
| 8 | Planar Pose Estimation | T. Collins, A. Bartoli (2014) | "Infinitesimal Plane-Based Pose Estimation," IJCV | Exact, fast pose of a planar square from its four corners (IPPE) | Our tag pose uses OpenCV `solvePnP` with `SOLVEPNP_IPPE_SQUARE` | Two-fold pose ambiguity at near-frontal views; solved by keeping the lower reprojection error |

### 3.2  Base paper math:

## Base Paper Equations (Li et al., 2019 — No Quaternions)

### 1. Pixel Conversion (Camera → Digital Image)

$$
u = \frac{x}{dx} + u_0 \qquad v = \frac{y}{dy} + v_0
$$

**In plain words:** A camera sensor sees the world as a continuous picture, but a computer stores it as a grid of pixels (like graph paper). This equation just says: "take the real position on the sensor and tell me which pixel box it lands in." (Paper Eq. 1–3.)

| Variable | What it is | Why it's needed |
|---|---|---|
| `x, y` | Where a point actually is on the camera sensor (in millimeters) | This is the "real" physical location before we turn it into pixels |
| `dx, dy` | How big one pixel is (in millimeters) | Tells us how many pixels fit into one millimeter — this does the mm → pixel conversion |
| `u, v` | The pixel address of that point (column, row) | This is what the computer actually works with — an image is just numbers in a pixel grid |
| `u0, v0` | The pixel at the exact center of the image | Cameras don't always have their center at pixel (0,0), so we shift by this amount to correct for that |

Combined with the pinhole model this gives $u=u_0+f\,X_c/Z_c$, with $f=F/dx$ in pixels. For our 640×480 camera with a 60° vertical field of view, $f=240/\tan30^\circ=415.7$ px.

---

## After this they use the AprilTag detector (Olson, ref. [12] of the paper), which finds the tag, computes a homography by DLT and recovers the tag pose. In our implementation the library `pupil_apriltags` identifies the AprilTag and OpenCV `solvePnP` (IPPE_SQUARE) recovers its pose, and then the pipeline continues with the steps below

### 2. One-Axis Rotation (Roll)

$$
R_x(\phi) =
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi & \cos\phi
\end{bmatrix}
$$

**In plain words:** Imagine tilting a tray left-to-right — that's roll. This matrix is a "tilt calculator": give it an angle, and it tells you how every point on the tray moves. (Paper Eq. 4–6. The paper labels this matrix $R_z$, but it is the rotation about x, and its Eq. 5 prints $y\sin\alpha$ where $y\cos\alpha$ is correct.)

| Variable | What it is | Why it's needed |
|---|---|---|
| `φ` (phi) | The roll angle — how much the drone/tag is tilted sideways | Without this, we can't tell how the object rotated around one axis |
| `R_x(φ)` | The 3×3 matrix itself | A reusable "rotation machine" — multiply it with a point and it spins that point by angle φ |

---

### 3. Full 3D Rotation (Combining Roll, Pitch, Yaw)

$$
R = R_z(\psi)\, R_y(\theta)\, R_x(\phi)
$$

**In plain words:** A drone doesn't just tilt in one direction — it rolls, pitches (nose up/down), and yaws (spins left/right) all at once. Chaining three simple tilt calculators together gives the *full* rotation. (Paper Eq. 7–8. Three entries of the printed Eq. 8 contain typos; e.g. entry (1,1) should be $\cos\theta\cos\psi$.)

| Variable | What it is | Why it's needed |
|---|---|---|
| `ψ` (psi) | Yaw angle — spinning left/right | Captures rotation around the vertical axis |
| `θ` (theta) | Pitch angle — nose up/down | Captures forward/backward tilt |
| `φ` (phi) | Roll angle — tilting side to side | Captures sideways tilt |
| `R` | The combined 3×3 rotation matrix | **This is the base paper's final answer for orientation** — no quaternion involved |

**Why it is a problem:** at $\theta=90^\circ$ the matrix depends only on $\phi-\psi$, so roll and yaw merge and one degree of freedom is lost (gimbal lock).

---

### 4. PID Controller (Turning Error into Speed Commands)

$$
V = K_p \cdot e \;+\; K_i \int e\, dt \;+\; K_d \cdot (e - e_{prev})
$$

**In plain words:** "How fast should the drone move to fix its position error?" It looks at three things — how far off you are *right now*, how far off you've *been* over time, and how *quickly* the error is changing — and blends all three into one speed command. (Paper Eq. 9, with published gains $K_p=0.20$, $K_i=0.03$, $K_d=0.35$.)

| Variable | What it is | Why it's needed |
|---|---|---|
| `e` | The current position error (how far the drone is from where it should be) | The main signal — no error means no correction needed |
| `Kp · e` | Proportional term — reacts to the error right now | Bigger error → bigger push toward the target |
| `Ki ∫e dt` | Integral term — adds up all past error over time | Corrects small, persistent errors (like drift) that proportional alone can't fix |
| `Kd(e − e_prev)` | Derivative term — reacts to how fast the error is changing | Acts like a brake — stops overshoot and oscillation |
| `Kp, Ki, Kd` | Tuning knobs (gains) for each term | Control how aggressive or gentle each correction is |
| `V` | The final speed command sent to the drone | The actual output that moves the drone |

**Limitation we found:** with a pad moving at $V_{pad}$, a PID can only produce speed from an error, so it settles behind the pad at $e=V_{pad}/K_p=0.9/0.20=4.5$ m. That is larger than the 1.5 m pad, so the drone never lands. Also, the printed D term is not divided by the sample time.

---

### 5. Position Error

$$
\text{Error} = \text{Desired Position} - \text{Current Position}
$$

**In plain words:** "Where you want to be" minus "where you actually are." That gap is what the PID controller above is trying to shrink to zero.

| Variable | What it is | Why it's needed |
|---|---|---|
| Desired Position | Where the drone should be (right above the pad centre) | The target — set by the landing task |
| Current Position | Where the drone actually is, from AprilTag pose estimation | Our best real-time estimate of the truth |
| Error | The gap between the two | Drives the entire PID control loop |

---

## Our Contribution (Not in the Base Paper)

The base paper stops at equation 3 — a plain rotation matrix — and hands attitude control to the DJI autopilot. **Everything below is our own addition**, applying quaternion theory from course 22AIE448.

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

In the controller itself we go straight from a rotation matrix to a quaternion (Shepperd's method). If $\text{tr}R>0$: $w=\tfrac12\sqrt{1+r_{11}+r_{22}+r_{33}}$, $x=(r_{32}-r_{23})/4w$, $y=(r_{13}-r_{31})/4w$, $z=(r_{21}-r_{12})/4w$. Otherwise the branch of the largest diagonal entry is used, so the formula never divides by a number near zero.

| Variable | What it is | Why it's needed |
|---|---|---|
| $r_{ij}$ | The element in row $i$, column $j$ of the rotation matrix $R$ | The raw ingredients — read directly from the rotation matrix |
| $c_x,s_x,c_y,s_y,c_z,s_z$ | Cosine and sine of the half roll, pitch and yaw angles | A quaternion stores *half* the angle, because it is applied twice in $q\,v\,q^*$ |
| $q_0 \ (= w)$ | The "how much rotation" part of the quaternion (the scalar/real part) | Tells you the *size* (angle) of the rotation: $w=\cos(\theta/2)$ |
| $q_1, q_2, q_3 \ (= x, y, z)$ | The "which direction" part of the quaternion (a 3D vector) | Together with $q_0$, fully describes the same rotation — just more compactly than a 3×3 matrix |
### B. Rotating a Vector with a Quaternion (Sandwich Operator)

$$
L_q(v) = q\, v\, q^{*}
$$

**In plain words:** Think of `v` as a small arrow (like "the tag is 2 meters that way"). To rotate that arrow using a quaternion, you "sandwich" it — multiply the quaternion on one side and its conjugate on the other. What comes out is the same arrow, just rotated. Example: rotating east $(1,0,0)$ by $q=[0.707,0,0,0.707]$ (90° about z) gives north $(0,1,0)$.



### Coordinate Frame Transformation & Sandwich Operator

The onboard camera measures position error in its own optical frame, but the flight controller requires this error expressed in the drone's **body frame** ($B$) and finally in the **world frame** ($W$). As the quadrotor tilts and pitches during flight, these coordinate frames no longer align.



Camera Frame (C)  ───[ Sandwich Operator: q v q⁻¹ ]───►  Body Frame (B)  ───►  PID Controller
(Raw Error Vector)                                     (Aligned Error)       (Actuator Commands)



#### Why the Sandwich Operator is Required

* **Quaternion Representation:** A unit quaternion $\mathbf{q}$ stores the 3D orientation/tilt of the vehicle, but a quaternion alone cannot rotate a 3D vector without an algebraic operator.
* **Double-Cover & Distortion Elimination:** Simple left-multiplication ($\mathbf{q}\mathbf{v}$) produces a four-dimensional result with unwanted scaling/projection artifacts. The **Sandwich Operator** maps a pure spatial vector back into the 3-vector space cleanly:

$$v' = q \, v \, q^{-1}$$

Where:
* $v = [0, \mathbf{e}_{cam}]^T$ is the raw visual position error treated as a pure vector quaternion.
* $q$ is the unit attitude quaternion representing current vehicle tilt.
* $q^{-1} = q^*$ is the quaternion conjugate (inverse for unit quaternions).
* $v' = [0, \mathbf{e}_{body}]^T$ is the transformed position error vector.

#### Integration in Control Loop

The resulting vector $v'$ represents the exact relative position error re-expressed in the drone's body frame:

$$\mathbf{e}_{body} = \text{vec}\left( q \, [0, \mathbf{e}_{cam}] \, q^{-1} \right)$$

The pad centre is then corrected for the 0.40 m offset of the large tag, which turns with the pad heading $\psi_{pad}$: $c_{pad}=p_{tag}-R_z(\psi_{pad})\,[0.40,\,0]^T$. This transformed error is routed into the **PID controller** to generate velocity commands without frame misalignment errors.


| Variable | What it is | Why it's needed |
|---|---|---|
| `q` | The quaternion describing the rotation (from step A) | The "rotation instruction" |
| `q*` | The conjugate of `q` — `q` flipped/reversed | Needed to correctly complete the rotation and leave a clean result |
| `v` | The vector being rotated — here, the tag's position | We want the tag's position *relative to the drone*, not the camera |
| `L_q(v)` | The final, rotated vector | The tag's position now expressed in the drone's own frame of reference |

### C. Quaternion Attitude Control (Thrust Vector → $q_{des}$ → Torque)

$$
f = m(a_{cmd}+g\,e_3),\qquad b_3=\frac{f}{\lVert f\rVert},\qquad b_2=\frac{b_3\times c_1}{\lVert b_3\times c_1\rVert},\qquad b_1=b_2\times b_3
$$

$$
q_{err}=q^*\otimes q_{des}\ \ (\text{flip if } w_{err}<0),\qquad e_R=2\,\mathrm{atan2}(\lVert\vec q_{err}\rVert,w_{err})\frac{\vec q_{err}}{\lVert\vec q_{err}\rVert},\qquad \tau=J(K_R\,e_R-K_\omega\,\omega)
$$

**In plain words:** to move sideways the drone must lean. The required force $f$ gives the new "up" axis $b_3$. For example, accelerating at 2 m/s² needs a lean of $\arctan(2/9.81)=11.5^\circ$. The error between where the drone points and where it should point is *one* quaternion product, and the sign check always picks the short way round: 179° → −179° is a 2° turn, not 358°. With $K_R=130$ and $K_\omega=22$ the attitude loop has $\omega_n=11.4$ rad/s and $\zeta=0.97$, so it settles with almost no overshoot.

| Variable | What it is | Why it's needed |
|---|---|---|
| $a_{cmd}$ | Acceleration requested by the velocity loop | What the drone must achieve |
| $b_3$ | Desired thrust ("up") axis | The propellers only push along the body's up axis |
| $q_{des}$ | Desired attitude quaternion, from $R_{des}=[b_1\,b_2\,b_3]$ | The orientation to reach |
| $q_{err}$, $e_R$ | Remaining rotation, as a quaternion and as axis × angle | What the torque must correct |
| $\tau$ | Body torque, with $T=f\cdot b_3^{now}$ the thrust | Sent to the motors |

### D. Guidance for Moving Pads (Feed-forward, Tracker, Decision Height)

$$
v_{cmd}=g(h)\,[K_p e+K_d\dot e]+K_i I+\hat v_{pad},\qquad \Omega=\frac{(\hat v\times\hat a)_z}{\lVert\hat v\rVert^2},\qquad h_{blind}=0.25+1.73\,e
$$

**In plain words:** we add the pad's own velocity $\hat v_{pad}$ (from an α-β-γ tracker with gains 0.35, 0.08 and 0.010) so the PID only fixes the small leftover. This removes the 4.5 m lag. The gain $g(h)$ rises from 1 above 4 m to 3 below 0.6 m, because near the ground the camera sees only a small patch. The tracker estimates the turn rate Ω and aims 0.25 s ahead along the curve. The drone descends only while $\lVert e\rVert<0.22\,h$. Below $h_{blind}$ even the small tag no longer fits in the image, so the drone commits to landing.


### 3.3  Base paper link:
[IEEE Xplore](https://ieeexplore.ieee.org/document/8865757) 

### 4.  Complete System Pipeline

<h2 align="center">Fig 4.1</h2>

<p align="center">
<img width="700" height="800" alt="image" src="https://github.com/user-attachments/assets/8f361a3d-0164-447e-81aa-852b44e49711" />
</p>

<h2 align="center">Fig 4.2</h2>

<p align="center">
<img width="1481" height="939" alt="mj_diag_pipeline" src="https://github.com/user-attachments/assets/50c967ce-959c-4c3e-9fcb-fe911cead88e" />
  
</p>

<p align="center"><i>Signal flow of the final system. Blue blocks are from the base paper, orange blocks are added by this project, and grey blocks are hardware and physics. The camera image of the moved drone closes the loop.</i></p>

<h2 align="center">Fig 4.3</h2>

<p align="center">
  <img width="1598" height="881" alt="Simulated street with ten addressed pads" src="https://github.com/user-attachments/assets/f0038acf-5a46-4a4c-888a-3216a2ff34be" />
</p>



<p align="center"><i>The simulated street: ten addressed pads (street, moving vehicles, roofs), and the onboard camera images at 8 m, 2 m and 0.45 m.</i></p>

### 5.Methodology

## Methodology

Our implementation is a full physics-based simulation of the extended pipeline, built in MuJoCo, with four modules working in a closed loop at each control step:

1. **Physics simulation (`drone_sim/physics/quad.py`)**
   - A MuJoCo model of a DJI M100-class quadrotor (3.5 kg, inertia diag(0.045, 0.045, 0.080) kg·m², thrust-to-weight 2.2).
   - Stepped at 500 Hz internally (`DT_SIM = 0.002 s`) and controlled at 100 Hz (`DT_CTRL = 0.01 s`).
   - Includes a 40 ms motor lag, a 25° tilt limit, linear drag and wind (a steady (0.6, −0.4) m/s plus correlated gusts with σ = 0.5 m/s).
   - The drone's orientation is kept as a quaternion `[qw, qx, qy, qz]` everywhere. It is never converted to Euler angles inside the loop.

2. **Perception (`drone_sim/perception/vision.py`, `beacon.py`)**
   - The drone's own downward camera (640×480, 60° FOV, f = 415.7 px, 25 Hz) is rendered by MuJoCo, so when the drone tilts the tag really swings across the image.
   - Every pad carries a large 0.42 m `tag36h11` tag (ID k, offset 0.40 m from the centre) and a small 0.12 m tag (ID k+10) at the centre.
   - `pupil_apriltags` decodes the tags, and OpenCV `solvePnP` with `IPPE_SQUARE` recovers each tag's pose from its four corners.
   - The pose is moved camera → body → world with the Sandwich Operator, corrected to the pad centre, and the pad heading is smoothed with SLERP.
   - Before vision lock, a radio beacon (AirTag-style, 2 Hz, ±2.5 m) guides the transit.

3. **Guidance (`drone_sim/control/guidance.py`, `drone_sim/mission.py`)**
   - A mission state machine: `TAKEOFF → TRANSIT → SEARCH → ALIGN → DESCEND → FINAL → LANDED`.
   - The paper's PID (0.20, 0.03, 0.35), extended with a filtered derivative, a leaky integral, pad-velocity feed-forward and the altitude gain schedule.
   - An α-β-γ coordinated-turn tracker supplies the pad's velocity and turn rate.
   - Descent is allowed only inside the cone $\lVert e\rVert<0.22h$. Below the decision height the drone commits to landing, with at most 2 go-arounds.

4. **Attitude control (`drone_sim/control/attitude.py`, `quaternion.py`)**
   - A velocity loop (gains 1.6 and 1.2) produces the required acceleration.
   - The thrust vector gives $q_{des}$, the quaternion error $q^*\otimes q_{des}$ gives the torque, and the thrust is $T=f\cdot b_3^{now}$.
   - These are applied back to MuJoCo, which closes the loop.

The same model (same equations, gains and pads) was then rebuilt independently in **MATLAB** (`matlab/simulate_landing.m`, 3-D animation `run_demo3d`) and **NetLogo** (`netlogo/drone_airtag_landing.nlogo`), to check that the results do not depend on one simulator.

This differs from the base paper in four important ways:

1. Attitude is represented and controlled with quaternions instead of Euler rotation matrices.
2. Pose estimation uses IPPE on the real rendered image.
3. The controller is phase-aware and handles moving and turning pads (feed-forward, tracker, gain schedule, decision height) instead of a single flat PID loop.
4. The drone lands on any of ten addressed pads, not one.



### 6.Methodology Block Diagram

<h2 align="center">Fig 6.1</h2>
<p align="center">
<img width="700" height="700" alt="image" src="https://github.com/user-attachments/assets/6e49ec3c-b354-4c10-9adc-d0728c1e1b22" />
</p>

<h2 align="center">Fig 6.2</h2>
<p align="center">
<img width="1539" height="558" alt="mj_diag_states" src="https://github.com/user-attachments/assets/b21ec3d3-f8c5-4b25-a0b8-d342b021a2d7" />

</p>

<p align="center"><i>Mission phases. Guidance switches from radio to vision exactly once, at the first decode of the addressed marker. If the tag is lost or the offset leaves the cone, the drone climbs and re-acquires.</i></p>

<h2 align="center">Fig 6.3</h2>
<p align="center">
<img width="1194" height="1116" alt="mj_diag_pad" src="https://github.com/user-attachments/assets/1024e020-0056-443b-8df1-e1d8198bbc7f" />

</p>

<p align="center"><i>One pad carries two markers of two sizes with two IDs. The large tag reads from 1.06 to 8.73 m and the small tag below 2.49 m.</i></p>

### 7.Results

<h3 align="center">Table 7.1 — Three platforms, one model (40 missions each)</h3>

| Platform | Landed | Mean error | Static pads | Moving pads | Worst address |
|---|---|---|---|---|---|
| MuJoCo (Python, real camera + detector) | 40 / 40 | 2.41 cm | 1.21 cm | 5.18 cm | 5 |
| MATLAB | 40 / 40 | 3.38 cm | 1.85 cm | 6.94 cm | 5 |
| NetLogo | 40 / 40 | 2.80 cm | 1.56 cm | 5.69 cm | 5 |
| **Total** | **120 / 120** | — | — | — | — |

<h3 align="center">Table 7.2 — What each addition is worth (MuJoCo ablation, 6 runs per row)</h3>

| Controller | Moving pads landed | Moving error | Static error |
|---|---|---|---|
| Base paper PID only | 0 / 6 | — | 10.9 cm |
| + integral anti-windup | 0 / 6 | — | 2.3 cm |
| + pad-velocity feed-forward | 4 / 6 | 8.9 cm | 1.8 cm |
| + altitude gain scheduling | 6 / 6 | 6.5 cm | 0.9 cm |
| + coordinated-turn tracker (full) | 6 / 6 | 3.1 cm | 0.6 cm |

Further results:
- MuJoCo 95th-percentile error: 8.34 cm.
- Mid-flight change of address: 20/20 landed, mean 2.94 cm, 0 go-arounds.
- SLERP heading smoothing: 2.7° → 1.6° heading noise, 1.9 → 1.1 cm pad-centre noise.

### Drone detecting apriltag stimulation in mujoco 

<img width="653" height="366" alt="mujoco_panel" src="https://github.com/user-attachments/assets/3976770c-3f0c-4fde-86e3-dfac9e1d1daf" />

<img width="480" height="269" alt="demo_landing" src="https://github.com/user-attachments/assets/dc90f950-ec85-4283-b07c-edfab6a17b50" />


### This picture shows the stimulation of drone detecting the apriltag in netlogo:

<img width="1550" height="903" alt="netlogo" src="https://github.com/user-attachments/assets/08ac1ee9-992d-4d4e-9333-96200f1f7fde" />

# monte carlo distribution

<img width="1652" height="764" alt="mj_fig5_monte_carlo" src="https://github.com/user-attachments/assets/3f3f3f66-0d58-41cf-8440-ee529f54a8bf" />




### 8.Conclusion

This project turned the single-tag, static-pad landing method of Li et al. (2019) into a system that lands on any of ten addressed pads, including rooftops and moving vehicles. The changes that made this possible were:

- dual-scale tags with a derived hand-over window and decision height
- quaternion attitude representation and control with the Sandwich Operator (no gimbal lock, no ±180° wrap-around)
- a feed-forward, gain-scheduled PID
- a coordinated-turn tracker

The method landed **120 / 120** times across three independent platforms, with mean errors of 2.4–3.4 cm, about 2% of the pad width. Moving-pad landings went from **0 / 6** with the paper's PID to **6 / 6**.

**Limitation:** address 5, a rover turning on a 0.9 m circle, is the hardest pad on every platform. Physics alone leaves about $\tfrac12\Omega^2 r t^2\approx 8$ cm of error. When the turn-rate estimate is still active at commit, the mean miss is 8.8 cm; when it has dropped out, it is 23.5 cm.

**Future work:**
- an Interacting Multiple Model (IMM) tracker
- real-flight tests with lighting changes and motion blur

<h3 align="center">Table 8.1</h3>

| # | Reference | Link |
|---|---|---|
| 1 | Z. Li, Y. Chen, H. Lu, H. Wu, L. Cheng, "UAV Autonomous Landing Technology Based on AprilTags Vision Positioning Algorithm," CCC 2019 | [IEEE Xplore](https://ieeexplore.ieee.org/document/8865757) |
| 2 | E. Olson, "AprilTag: A Robust and Flexible Visual Fiducial System," ICRA 2011 | [IEEE Xplore](https://ieeexplore.ieee.org/document/5979561) · [Free PDF (UMich)](https://april.eecs.umich.edu/pdfs/olson2011a.pdf) |
| 3 | E. Fresk, G. Nikolakopoulos, "Full Quaternion Based Attitude Control for a Quadrotor," ECC 2013 | [Google Scholar](https://scholar.google.com/scholar?q=Full+Quaternion+Based+Attitude+Control+for+a+Quadrotor+Fresk+Nikolakopoulos) |
| 4 | D. Falanga, A. Zanchettin, A. Simovic, J. Delmerico, D. Scaramuzza, "Vision-Based Autonomous Quadrotor Landing on a Moving Platform," SSRR 2017 | [IEEE Xplore](https://ieeexplore.ieee.org/document/8088164) · [Free PDF (UZH RPG)](http://rpg.ifi.uzh.ch/docs/SSRR17_Falanga.pdf) |
| 5 | T. Lee, M. Leok, N. H. McClamroch, "Geometric Tracking Control of a Quadrotor UAV on SE(3)," CDC 2010 | [DOI](https://doi.org/10.1109/CDC.2010.5717652) |
| 6 | K. Shoemake, "Animating Rotation with Quaternion Curves," SIGGRAPH 1985 | [DOI](https://doi.org/10.1145/325165.325242) |
| 7 | Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley 2001 | [DOI](https://doi.org/10.1002/0471221279) |
| 8 | T. Collins, A. Bartoli, "Infinitesimal Plane-Based Pose Estimation," IJCV 2014 | [DOI](https://doi.org/10.1007/s11263-014-0725-5) |
| 9 | E. Todorov, T. Erez, Y. Tassa, "MuJoCo: A Physics Engine for Model-Based Control," IROS 2012 | [DOI](https://doi.org/10.1109/IROS.2012.6386109) |
| 10 | U. Wilensky, NetLogo, Northwestern University, 1999 | [NetLogo](https://ccl.northwestern.edu/netlogo/) |


