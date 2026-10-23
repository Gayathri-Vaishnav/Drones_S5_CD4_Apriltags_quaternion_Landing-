# Autonomous Landing of Quadrotor UAV Using AprilTags Vision Positioning and Quaternion-based Attitude Representation

**Project basis:** *Mathematics_of_the_Landing* presentation and *Final Project Report: Vision-Guided Autonomous UAV Landing on Addressed, Moving Targets. It covers the five core topics of the project, explaining the underlying mathematical concepts, equations, and their applications in autonomous drone landing. The explanations follow the terminology and methods presented in the source materials, with additional clarification where necessary to improve understanding.


## The full mathematical pipeline

1. **Seeing the tag:** camera pixels → 3D tag pose → pad-centre position in world coordinates.
2. **Predicting a moving pad:** estimate position, velocity, acceleration, and (when appropriate) turn rate.
3. **Deciding where to fly:** position error → desired velocity using PID, plus moving-target feed-forward and safety limits.
4. **Finding the pad's pose:** use quaternion rotations to transform positions and represent pad/drone orientation.
5. **Tilting the drone:** desired force → desired attitude → quaternion attitude error → body torque and thrust.

The presentation says the camera/perception loop runs at **25 Hz**, while the attitude/control loop runs at **100 Hz**. A prediction can run at every control step, but a measurement correction should happen only when a new valid camera detection is available.

---

# 1. Seeing the tag — camera geometry and pad position

## 1.1 Pinhole camera model: focal length in pixels

The presentation gives a camera resolution of \(640 \times 480\) pixels and a vertical field of view (FOV) of \(60^\circ\).

For image height \(H_{\rm px}\) and vertical field of view \(\phi_y\),

\[
\boxed{f_y=\frac{H_{\rm px}/2}{\tan(\phi_y/2)}}
\]

Substitute the project's values:

\[
f_y=\frac{480/2}{\tan(60^\circ/2)}
=\frac{240}{\tan 30^\circ}
\approx \boxed{415.7\text{ pixels}}
\]

If square pixels are assumed, \(f_x\approx f_y\). More generally, use the horizontal FOV to compute \(f_x\).

**What this means:** focal length is being expressed in pixels, not millimetres. It relates how large an object appears in the image to its distance from the camera.

**Why used:** the detector sees pixel coordinates, while guidance needs distances in metres. Camera intrinsics connect those two representations.


## 1.2 Project a 3D point into the image

For a point in the camera coordinate frame \((X_c,Y_c,Z_c)\), the ideal pinhole projection is

\[
u=c_x+f_x\frac{X_c}{Z_c},\qquad
v=c_y+f_y\frac{Y_c}{Z_c}
\]

where:
- \((u,v)\): image pixel coordinates;
- \((c_x,c_y)\): principal point, approximately the image centre;
- \(f_x,f_y\): focal lengths in pixels;
- \(Z_c\): depth along the camera's optical axis.

The inverse relationship, when depth is known, is

\[
X_c=\frac{(u-c_x)Z_c}{f_x},\qquad
Y_c=\frac{(v-c_y)Z_c}{f_y}.
\]

**Why this is not enough by itself:** a single pixel gives a viewing ray, not a unique 3D point, because depth is unknown. The project uses a marker of known physical size and its four corners to recover pose.

## 1.3 Recover the marker pose using four corners

The project passes the four detected corners of a square AprilTag, plus its known physical edge length, into OpenCV `solvePnP` (the presentation specifies `IPPE_SQUARE`).

Conceptually, each known 3D corner \(P_i\) in the marker frame is mapped into the camera frame by

\[
P_{c,i}=R_{ct}P_{t,i}+t_{ct}
\]

and then projected into pixels using the pinhole model:

\[
\tilde p_i \sim K\,[R_{ct}\mid t_{ct}]\,\tilde P_{t,i}.
\]

Here:
- \(R_{ct}\): rotation from tag coordinates to camera coordinates;
- \(t_{ct}\): translation from tag coordinates to camera coordinates;
- \(K\): camera intrinsic matrix;
- \(\sim\): equality up to homogeneous scale.

`solvePnP` finds the rotation and translation that best explain the observed corner pixels. The resulting translation is the tag's 3D position relative to the camera.

**Why four corners?** The known square geometry constrains both apparent scale (depth) and orientation (tilt/rotation). Pose recovery is more informative than simply measuring the tag's pixel width.

## 1.4 Convert camera-frame position into world coordinates

The presentation writes the transformation in the form

\[
\boxed{p_{\rm world}=R(q)\,C\,t_{\rm cam}}
\]

where \(R(q)\) is the rotation matrix from the drone's attitude quaternion and \(C=\operatorname{diag}(1,-1,-1)\) is the stated camera/body axis-convention conversion.

For a fully general transform, camera translation relative to the world origin must also be included:

\[
\boxed{p_{\rm tag}^{\,W}=p_{\rm cam}^{\,W}
+R_{WB}\,R_{BC}\,p_{\rm tag}^{\,C}}
\]

- \(p_{\rm tag}^{C}\): tag translation measured in camera coordinates;
- \(R_{BC}\): camera-to-body rotation;
- \(R_{WB}\): body-to-world rotation;
- \(p_{\rm cam}^{W}\): camera origin in world coordinates.

The shorter slide equation assumes the relevant origin/translation is already accounted for in the implementation. **Coordinate-frame conventions and multiplication order must match the code.** A sign or frame-order error can mirror or rotate the estimated target.

## 1.5 Correct tag position to get the pad centre

The report says the large tag is offset from the pad centre by \(0.40\) m. Its offset must rotate with the pad heading.

The presentation gives

\[
\boxed{c_{\rm pad}=p_{\rm tag}-R_z(\psi_{\rm pad})
\begin{bmatrix}0.40\\0\end{bmatrix}}
\]

for a planar horizontal offset, where

\[
R_z(\psi)=
\begin{bmatrix}
\cos\psi&-\sin\psi\\
\sin\psi&\cos\psi
\end{bmatrix}.
\]

Expanded:

\[
\begin{aligned}
x_{\rm pad}&=x_{\rm tag}-0.40\cos\psi_{\rm pad},\\
y_{\rm pad}&=y_{\rm tag}-0.40\sin\psi_{\rm pad}.
\end{aligned}
\]

The exact signs depend on whether the stored offset is defined from the pad centre to the tag or vice versa; the formula above follows the presentation's stated convention.

**Why needed:** if the tag is not at the centre, landing on the tag is not the same as landing on the pad centre. For a moving/turning vehicle, the offset rotates with its heading.

## 1.6 Maximum detection height

The project assumes a marker needs to span at least \(p_{\min}=20\) pixels to decode. Under a fronto-parallel pinhole approximation, a marker of edge \(s\) at distance \(h\) spans approximately

\[
p\approx \frac{f_y s}{h}.
\]

Set \(p=p_{\min}\) to obtain the maximum detection distance/height:

\[
\boxed{h_{\max}\approx\frac{f_y s}{p_{\min}}}
\]

For the large marker, \(s=0.42\) m:

\[
h_{\max}=\frac{415.7(0.42)}{20}
\approx\boxed{8.73\text{ m}}.
\]

For the small marker, \(s=0.12\) m:

\[
h_{\max}=\frac{415.7(0.12)}{20}
\approx\boxed{2.49\text{ m}}.
\]

These are approximate geometric limits, not guarantees of detection in every image.

## 1.7 Why two marker sizes?

The report/presentation uses:
- large marker edge: \(0.42\) m;
- small marker edge: \(0.12\) m;
- large marker leaves the image at close range; the slide reports a frame-limit height around \(1.06\) m;
- both markers are usable in the overlap window \(1.06\)–\(2.49\) m.

A large tag is visible from higher up but can become too large to fit in the camera frame close to the ground. A small tag fits near touchdown but cannot be decoded from as far away. The overlap window permits a controlled handover.

The presentation also gives an empirically measured decision-height relationship:

\[
\boxed{h_{\rm blind}(e)=0.25+1.73e}
\]

where \(e\) is the horizontal offset used by that fit. Treat this as a measured model from the presentation, **not a universal camera law**. Its units and the precise definition of \(e\) must be kept consistent with the experiment.


---

# 2. Predicting a moving pad — alpha-beta-gamma tracking and turning motion

## 2.1 Why tracking is required

Camera detections arrive at 25 Hz, while control runs at 100 Hz. Between camera frames, and especially during the final blind portion of descent, the pad can move. The tracker estimates:

\[
\hat p,\qquad \hat v,\qquad \hat a
\]

where \(\hat p\) is position, \(\hat v\) velocity, and \(\hat a\) acceleration.

The presentation says to **predict every control step** but **correct only when a new valid camera detection arrives**. Reusing one held camera measurement as if it were new at 100 Hz would bias the velocity estimate toward zero.

## 2.2 Prediction step: constant acceleration

Let \(\Delta t\) be the elapsed time since the last prediction. The standard alpha-beta-gamma predictor is

\[
\begin{aligned}
p^-_k &= \hat p_{k-1}+\hat v_{k-1}\Delta t
+\frac12\hat a_{k-1}\Delta t^2,\\
v^-_k &= \hat v_{k-1}+\hat a_{k-1}\Delta t,\\
a^-_k &= \hat a_{k-1}.
\end{aligned}
\]

The superscript \((-)\) means **predicted before using the new measurement**.

**What it does:** estimates where the pad should be now, based on its previous position, velocity and acceleration.

## 2.3 Residual (innovation)

If a valid vision measurement \(z_k\) arrives, calculate

\[
\boxed{r_k=z_k-p^-_k}
\]

- \(z_k\): measured pad position;
- \(p^-_k\): predicted position;
- \(r_k\): prediction error/residual.

A positive residual means the observed position is in the positive direction relative to the prediction.

## 2.4 Correction step: alpha-beta-gamma filter

The project slide gives gains approximately

\[
\alpha=0.35,\qquad \beta=0.08,\qquad \gamma=0.010.
\]

A standard correction form is

\[
\begin{aligned}
\hat p_k&=p^-_k+\alpha r_k,\\
\hat v_k&=v^-_k+\frac{\beta}{\Delta t}r_k,\\
\hat a_k&=a^-_k+\frac{2\gamma}{\Delta t^2}r_k.
\end{aligned}
\]

**Important:** implementations sometimes absorb \(\Delta t\) into the gains. The equations above are the time-normalised form; verify the code's exact convention before claiming that the numeric gains are used in this exact form.

**Interpretation of gains:**
- \(\alpha\) controls how strongly position follows the new measurement;
- \(\beta\) controls how much residual changes estimated velocity;
- \(\gamma\) controls how much residual changes estimated acceleration.

Larger gains react faster but can pass more measurement noise into the estimate. Smaller gains smooth noise but respond more slowly to sudden changes.

## 2.5 Why acceleration matters

A constant-velocity model predicts a straight line:

\[
p(t+\tau)\approx p(t)+v(t)\tau.
\]

A constant-acceleration model adds a curvature/acceleration term:

\[
p(t+\tau)\approx p(t)+v(t)\tau+\frac12a(t)\tau^2.
\]

For a turning pad, velocity direction changes. The acceleration estimate helps predict the curved trajectory rather than extending the current tangent forever.

## 2.6 Turn rate from velocity and acceleration

For planar motion, the signed angular turn rate can be estimated from the 2D velocity and acceleration:

\[
\boxed{\omega=\frac{(\mathbf v\times\mathbf a)_z}{\|\mathbf v\|^2}}
\]

For vectors \(\mathbf v=(v_x,v_y,0)\) and \(\mathbf a=(a_x,a_y,0)\),

\[
\omega=\frac{v_xa_y-v_ya_x}{v_x^2+v_y^2}.
\]

### Notations

- $\omega$ — Turn rate (rad/s)
- $v_x$ — Velocity component along the x-axis
- $v_y$ — Velocity component along the y-axis
- $a_x$ — Acceleration component along the x-axis
- $a_y$ — Acceleration component along the y-axis

The cross product measures the component of acceleration perpendicular to velocity. Acceleration parallel to velocity changes speed; perpendicular acceleration changes direction.

### Why a gate is needed

The presentation/report uses a perpendicularity confidence measure:

\[
\boxed{\rho=\frac{\|\mathbf v\times\mathbf a\|}
{\|\mathbf v\|\,\|\mathbf a\|}}
\]

For planar nonzero vectors, \(\rho=|\sin\theta|\), where \(\theta\) is the angle between velocity and acceleration:
- \(\rho\approx1\): acceleration is almost perpendicular to velocity, consistent with turning;
- \(\rho\approx0\): acceleration is almost parallel/antiparallel, consistent with speeding up or braking in a straight line.

The report says that below a threshold of **0.55**, the system does not trust the turn estimate. The exact implementation should also guard against very small \(\|\mathbf v\|\) or \(\|\mathbf a\|\), which make the ratio unstable/undefined.

## 2.7 Predict along an arc: coordinated-turn model

For a constant turn rate \(\omega\) over \(\tau\), velocity rotates by angle \(\theta=\omega\tau\):

\[
\begin{bmatrix}v_x'\\v_y'\end{bmatrix}
=
\begin{bmatrix}
\cos\theta&-\sin\theta\\
\sin\theta&\cos\theta
\end{bmatrix}
\begin{bmatrix}v_x\\v_y\end{bmatrix}.
\]

For \(\omega\ne0\), the corresponding constant-speed position increment is

\[
\begin{aligned}
\Delta x&=\frac{v_x\sin(\omega\tau)+v_y(\cos(\omega\tau)-1)}{\omega},\\
\Delta y&=\frac{v_x(1-\cos(\omega\tau))+v_y\sin(\omega\tau)}{\omega}.
\end{aligned}
\]

As \(\omega\to0\), use the straight-line limit:

\[
\Delta x\to v_x\tau,\qquad \Delta y\to v_y\tau.
\]

This avoids dividing by a near-zero turn rate.



---

# 3. Deciding where to fly — PID guidance and feed-forward

## 3.1 Define the position error

Let the estimated pad centre be \(p_{\rm pad}\) and drone position be \(p_{\rm drone}\). The horizontal position error is

\[
\boxed{e=p_{\rm pad}-p_{\rm drone}}
\]

For horizontal guidance,

\[
e=\begin{bmatrix}e_x\\e_y\end{bmatrix}.
\]

With this convention, positive error means the pad is in the positive direction from the drone. If a codebase uses the opposite convention, controller signs must change accordingly.

## 3.2 PID control law in the base paper

The report quotes the base paper's Eq. (9) in discrete shorthand:

\[
\boxed{v_{\rm PID}=K_p e+K_i\int e\,dt+K_d(e-e_{\rm last})}
\]

with published gains \(K_p=0.20,\ K_i=0.03,\ K_d=0.35\).

The three terms are:
- **Proportional (P):** reacts to the current offset.
- **Integral (I):** accumulates past offset and can remove persistent bias.
- **Derivative (D):** reacts to how quickly the error changes and can damp the response.

The paper's printed difference term is not divided by the sample period. A rate-normalised derivative is

\[
\dot e_k\approx\frac{e_k-e_{k-1}}{\Delta t}.
\]

A practical discrete PID form is

\[
v_{\rm PID,k}=K_p e_k+K_i I_k+K_d\dot e_k,
\qquad I_k=I_{k-1}+e_k\Delta t.
\]

A low-pass filter is commonly applied to the derivative because differencing noisy vision measurements amplifies high-frequency noise.

**Important distinction:** the base paper's printed Eq. (9) and the project's rate-normalised/filtered implementation are not identical. The report says the project retained the published gains but changed the derivative implementation and added other safeguards.

## 3.3 Why PID alone lags behind a moving pad

If the pad moves at a steady velocity, a controller that only reacts to position error must first develop an offset before it commands the motion needed to follow it. This can create steady tracking lag.

The project adds feed-forward from the tracker's estimate of target motion:

\[
\boxed{v_{\rm cmd}=v_{\rm PID}+\hat v_{\rm pad}}
\]

and uses acceleration lead for turning/accelerating targets:

\[
\boxed{v_{\rm cmd}\approx v_{\rm PID}+\hat v_{\rm pad}+\tau_L\hat a_{\rm pad}}
\]

where the report describes an acceleration lead horizon of about \(\tau_L=0.25\) s.

**Intuition:** PID corrects the relative offset; feed-forward tells the drone how fast the pad itself is already moving. Acceleration lead anticipates the target's near-future change in velocity.

The exact vector saturation and frame conversion should follow the code's implementation.

## 3.4 Altitude gain scheduling

The report says the same proportional gain can be too slow high/low depending on the desired response: near the deck the camera sees a much smaller ground region and a moving pad can leave the frame quickly. The project schedules the gain according to height above the surface the pad sits on, rather than height above the street.

A generic representation is

\[
\boxed{K_p=K_p(h)}
\]

where \(h\) is height above the target deck. The slide says the proportional gain is increased by about a factor of three near touchdown relative to the cruise value in the described schedule. The exact piecewise schedule should be taken from the implementation/configuration; the presentation alone does not show all its breakpoints.

**Why deck-relative height?** A roof pad is physically above the street. The final landing behaviour should depend on distance to the pad's surface, not the absolute height above the road.

## 3.5 Integral windup and anti-windup

An integral term can keep growing while the commanded velocity is saturated or while the drone is far from the pad. Later, the stored integral may push in the wrong direction even after the current error has changed sign. This is called **integral windup**.

A generic leaky integral is

\[
I_k=\lambda I_{k-1}+e_k\Delta t,\qquad 0<\lambda\le1.
\]

For continuous-time leak constant \(T_I\), a common form is

\[
\dot I=e-\frac{1}{T_I}I
\]

when the leak is active. The report describes a 0.6 s time constant for bleeding off stale integral when it fights the current error, and freezing integration outside a narrow error band. The exact discrete update is implementation-specific.

**What anti-windup does in this project:** prevents an integral accumulated during approach from dominating the proportional correction near touchdown. The presentation's example reports a stale integral contribution of \(-0.095\) m/s opposing a proportional contribution of \(+0.062\) m/s.

## 3.6 Approach cone / descent safety gate

The presentation states that the horizontal offset must be less than \(0.22\) times height above the deck:

\[
\boxed{\|e\|<0.22h}
\]

This defines an approach cone: as \(h\) decreases, the allowed horizontal error shrinks. Descent is allowed only when the drone is sufficiently centred. The report also describes interlocks for live detections, blind descent and reacquisition.

**Why use a cone?** A 20 cm offset is relatively small at 5 m but large when only 30 cm above the pad. A height-scaled tolerance expresses that safety requirement.

## 3.7 Velocity loop versus position loop

The outer guidance loop turns target position error into a desired velocity. A lower-level velocity/acceleration loop then makes the drone follow that command. Do not confuse:
- \(e\): position error, usually metres;
- \(v_{\rm cmd}\): velocity command, metres/second;
- \(a_{\rm cmd}\): acceleration command, metres/second²;
- torque: rotational input, typically N·m.



---

# 4. Finding the pad's position — pose, quaternions and SLERP

This section concerns both **position** (where the pad centre is) and **orientation/pose** (how it is rotated). A pose combines translation and orientation.

## 4.1 Axis-angle representation of a quaternion

A rotation by angle \(\theta\) about a unit axis \(\hat n=(n_x,n_y,n_z)\) can be represented by the unit quaternion

\[
\boxed{q=\left[\cos\frac{\theta}{2},\ \hat n\sin\frac{\theta}{2}\right]}
\]

or in scalar-vector form \(q=[w,\mathbf v]\), where

\[
w=\cos(\theta/2),\qquad \mathbf v=\hat n\sin(\theta/2).
\]

The unit constraint is

\[
\boxed{\|q\|=1}.
\]

The identity/no-rotation quaternion is \(q=[1,0,0,0]\).

**Why use quaternions?** They avoid Euler-angle yaw wrap-around and the Euler representation's gimbal-lock singularity. Four numbers represent a 3D orientation subject to the unit constraint.

## 4.2 Quaternion conjugate and inverse

For \(q=[w,x,y,z]\), the conjugate is

\[
\boxed{q^*=[w,-x,-y,-z]}.
\]

For a unit quaternion,

\[
\boxed{q^{-1}=q^*}.
\]

The conjugate reverses the rotation. For a non-unit quaternion, the inverse is \(q^{-1}=q^*/\|q\|^2\), not just the conjugate.

## 4.3 Hamilton product

For \(q_1=[w_1,x_1,y_1,z_1]\) and \(q_2=[w_2,x_2,y_2,z_2]\), the Hamilton product \(q=q_1\otimes q_2\) is

\[
\begin{aligned}
w&=w_1w_2-x_1x_2-y_1y_2-z_1z_2,\\
x&=w_1x_2+x_1w_2+y_1z_2-z_1y_2,\\
y&=w_1y_2-x_1z_2+y_1w_2+z_1x_2,\\
z&=w_1z_2+x_1y_2-y_1x_2+z_1w_2.
\end{aligned}
\]

Quaternion multiplication is **not commutative**: in general, \(q_1\otimes q_2\ne q_2\otimes q_1\). Rotation order matters.

## 4.4 Rotate a vector using a quaternion

Represent a 3D vector \(\mathbf p\) as a pure quaternion \([0,\mathbf p]\). Then

\[
\boxed{[0,\mathbf p'] = q\otimes[0,\mathbf p]\otimes q^*}
\]

so \(\mathbf p'\) is the rotated vector. This is often called the quaternion sandwich product.

**Why the conjugate on the right?** It makes the operation apply the rotation represented by \(q\) while returning a pure-vector quaternion.

## 4.5 Quaternion sign ambiguity and shortest path

The quaternions \(q\) and \(-q\) represent the same physical orientation. This is important when interpolating or calculating attitude error.

To choose the shortest rotational path between two quaternions, if

\[
q_a\cdot q_b<0,
\]

replace \(q_b\) with \(-q_b\) before interpolation. This chooses the equivalent representation that lies closer on the unit-quaternion sphere.

## 4.6 SLERP: spherical linear interpolation

The project uses SLERP to smooth the measured heading/orientation. Define

\[
\cos\Omega=q_a\cdot q_b.
\]

After sign correction if the dot product is negative,

\[
\boxed{\operatorname{slerp}(q_a,q_b,t)=
\frac{\sin((1-t)\Omega)}{\sin\Omega}q_a+
\frac{\sin(t\Omega)}{\sin\Omega}q_b,\quad 0\le t\le1.}
\]

Here \(t\) is the interpolation fraction. For nearly identical quaternions, a numerically stable implementation may use a normalized linear interpolation limit.

**Why not ordinary averaging?** A straight average can leave the unit-quaternion sphere and can choose the long way around because \(q\) and \(-q\) represent the same rotation.

The report notes that the large tag's \(0.40\) m offset makes heading accuracy important: heading error becomes a pad-centre position error through that lever arm. The presentation reports heading noise falling from 2.7° to 1.6° and resulting position noise from 1.9 cm to 1.1 cm after SLERP.



---

# 5. Tilting the drone — desired attitude, attitude error, torque and thrust

## 5.1 Why the drone must tilt

A quadrotor's total thrust acts along its body thrust axis (the project describes it as body \(z\), with sign depending on convention). It cannot create arbitrary sideways thrust directly. To accelerate horizontally, it tilts the thrust direction.

The controller chain is:

\[
\boxed{\text{velocity error}\rightarrow a_{\rm cmd}
\rightarrow R_{\rm des}\rightarrow q_{\rm des}
\rightarrow q_{\rm err}\rightarrow\tau}
\]

where \(R_{\rm des}\) is desired rotation matrix, \(q_{\rm des}\) desired attitude quaternion, and \(\tau\) body torque.

## 5.2 Desired force from acceleration

Using the report's convention, the desired force is written

\[
\boxed{f=m(a_{\rm cmd}+g e_3)}
\]

where \(m\) is mass, \(g\) gravitational acceleration, and \(e_3\) is the vertical unit vector in the chosen world convention. If the world \(z\) axis points downward, the gravity sign must be adjusted accordingly. The physical idea is to combine desired acceleration with gravity compensation.

Normalize the desired force to obtain the desired body thrust-axis direction:

\[
\boxed{b_3=\frac{f}{\|f\|}}.
\]

The report describes choosing a horizontal heading reference \(c_1\), then forming the remaining body axes using cross products and normalization. One common consistent construction is

\[
b_2=\frac{b_3\times c_1}{\|b_3\times c_1\|},\qquad
b_1=b_2\times b_3.
\]

Then form the desired rotation matrix

\[
\boxed{R_{\rm des}=[\,b_1\ b_2\ b_3\,]}
\]

and convert \(R_{\rm des}\) to the desired quaternion \(q_{\rm des}\).

**Important:** cross-product order determines signs and handedness. Follow the report/code convention exactly; the construction above is one standard convention, not a replacement for checking the project's implemented order.

## 5.3 Attitude error quaternion

Let \(q\) be the current attitude and \(q_{\rm des}\) the desired attitude. The report defines the body-frame error as

\[
\boxed{q_{\rm err}=q^{-1}\otimes q_{\rm des}
=q^*\otimes q_{\rm des}}
\]

for unit \(q\). This is the relative rotation still needed to move from the current attitude to the desired one, under the project's multiplication convention.

If the scalar component \(w_{\rm err}<0\), replace the error quaternion by its negative:

\[
q_{\rm err}\leftarrow -q_{\rm err}.
\]

Because \(q_{\rm err}\) and \(-q_{\rm err}\) represent the same rotation, this makes the controller use the shortest-angle representation.

## 5.4 Convert error quaternion to a rotation vector

Write \(q_{\rm err}=[w,\mathbf v]\). The rotation angle and axis are

\[
\theta=2\operatorname{atan2}(\|\mathbf v\|,w),
\qquad
\hat n=\frac{\mathbf v}{\|\mathbf v\|}.
\]

The rotation vector is

\[
\boxed{e_R=\theta\hat n
=\frac{2\operatorname{atan2}(\|\mathbf v\|,w)}
{\|\mathbf v\|}\mathbf v}.
\]

For very small \(\|\mathbf v\|\), use the small-angle limit to avoid division by zero. This vector expresses the correction axis and correction angle together.

## 5.5 Torque control

The report gives the attitude controller in the form

\[
\boxed{\tau=J(K_p e_R-K_d\omega)}
\]

where:
- \(J\): rotational inertia matrix;
- \(e_R\): attitude rotation-error vector;
- \(\omega\): measured body angular velocity;
- \(K_p\): attitude-error gain;
- \(K_d\): angular-rate damping gain.

The proportional term turns the drone toward the desired orientation. The rate term damps rotation and reduces overshoot.

The report says its implementation uses \(J\cdot10\cdot(13e_R-2.2\omega)\) for roll and pitch, with smaller gains for yaw. Do not assume this exact scalar expression applies identically to all axes unless the code confirms it.

## 5.6 Thrust projection: use the current axis

The desired force vector tells the controller what force would be needed at the target attitude. But the drone is still rotating toward that attitude. Therefore thrust magnitude must be projected onto the **current** thrust axis, not simply set to the norm of the desired force.

The report gives

\[
\boxed{T=f\cdot b_3^{\rm now}}
\]

with the current body thrust-axis direction written as \(b_3^{\rm now}=R(q)e_3\) under its convention. If the model defines thrust in the opposite direction, the sign convention changes.

**Why it matters:** using the full desired-force magnitude while the drone is still tilted differently can create excess vertical force and an unintended climb. The report says correcting this projection reduced altitude excursion during a six-metre repositioning to under 3 cm in the simulated setup.

## 5.7 Quaternion kinematics and rigid-body motion

The report lists the translational dynamics in the general form

\[
\dot p=v,\qquad
m\dot v=T b_3-mg e_3+c_d(w-v),
\]

where \(c_d(w-v)\) is the project's drag/wind-relative-velocity term. Its sign and exact coefficient depend on the model's definitions.

Quaternion kinematics are given as

\[
\boxed{\dot q=\frac12\,q\otimes[0,\omega]}
\]

under the report's quaternion/angular-velocity convention. Some conventions place the angular-velocity quaternion on the left; the order must match the frame in which \(\omega\) is expressed.

Rotational dynamics are

\[
\boxed{J\dot\omega=\tau}
\]

in the simplified model presented. A more complete rigid body equation can include the gyroscopic term \(\omega\times J\omega\); do not claim it is included unless the implementation does include it.

After each numerical integration step, the quaternion is renormalized:

\[
\boxed{q\leftarrow\frac{q}{\|q\|}}
\]

to counter numerical drift away from unit length.

# 6. Conclusion

This project explores the mathematical principles behind vision-guided drone landing using camera geometry, AprilTag detection, PID guidance, moving-pad tracking, pose estimation, quaternion mathematics, and attitude control. These components work together to estimate the landing pad’s position, predict its movement, calculate the required control commands, and adjust the drone’s orientation. The reported simulation results demonstrate improved landing performance through anti-windup, velocity feed-forward, altitude gain scheduling, and coordinated-turn tracking. However, challenging turning trajectories and visual information loss near touchdown remain limitations. Future improvements could include more robust motion estimation and an interacting multiple model (IMM) filter to improve tracking under different movement patterns.





