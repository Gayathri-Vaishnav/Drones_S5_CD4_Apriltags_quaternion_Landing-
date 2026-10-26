# Autonomous Landing of a Quadrotor UAV Using AprilTags and Quaternions



---

## Workflow Overview

![pipeline](md_images/00_pipeline.png)

| Step | Question it answers | Input → output |
|---|---|---|
| 1. **Seeing the tag** | Where is the pad? | camera pixels → pad position in metres |
| 2. **Predicting the pad** | Where will the pad be? | positions → speed, acceleration, turn rate |
| 3. **Deciding where to fly** | How fast should I fly? | distance to pad → velocity command |
| 4. **Pose and quaternions** | Which way is everything pointing? | rotations, heading of the pad |
| 5. **Tilting the drone** | How do I make that velocity happen? | velocity → tilt → motor torques |

- The **camera** gives a new picture **25 times a second** (25 Hz).

- Between two pictures the drone **predicts**. When a new picture arrives, it **corrects** the prediction.

> **Real-life example:** a delivery rider. Their **eyes** read the door number (1). They notice the customer **walking towards them** (2). They **steer** towards the door (3). They know **which way they are facing** (4). Their **body leans** to turn the bike (5).

![the simulated world](figures/mj_fig0_scene.png)
The simulated street: 10 addressed pads on the ground, on moving vehicles and on roofs, plus what the drone's camera sees at 8 m, 2 m and 0.45 m.

---

# 1. Seeing the tag: camera geometry and pad position

## 1.1 The pinhole camera: "far things look small"

 **In simple words:** a camera turns a 3-D point into a pixel. The rule is just **divide by the distance**.
 **Real-life example:** hold your phone over a book on the floor. Lift the phone twice as high and the book looks half as big.

![pinhole](md_images/01_pinhole.png)

Our camera is 640 × 480 pixels, with a vertical field of view of 60°:

$$
f=\frac{H_{px}/2}{\tan(\text{FOV}/2)}=\frac{240}{\tan 30^\circ}=\boxed{415.7\ \text{pixels}}
$$

A point $(X_c, Y_c, Z_c)$ in front of the camera appears at pixel

$$
u=c_x+f\frac{X_c}{Z_c},\qquad v=c_y+f\frac{Y_c}{Z_c}
$$

| Symbol | Meaning |
|---|---|
| $u, v$ | pixel position in the picture |
| $c_x, c_y = 320, 240$ | centre of the picture |
| $f = 415.7$ px | focal length, measured in pixels |
| $X_c, Y_c$ | how far the point is to the side (m) |
| $Z_c$ | how far the point is in front; for our downward camera this is the **height** |

**Toy example.** The drone is **4 m** high and the tag is **0.5 m** to the right:

$$
u = 320 + 415.7\times\frac{0.5}{4} = 320 + 52 = 372\ \text{px}
$$

Reading the formula backwards gives the distance in metres: $X_c=(372-320)\times 4/415.7=0.50$ m ✔

**One pixel is not enough.** One pixel only tells us a *direction*, not a distance. That is why we use the tag's **four corners** and its **known size** (next section).

## 1.2 The tag's pose from its four corners

**How it works:** we know the tag is a square with edges 0.42 m long. We see where its 4 corners land in the picture. There is only **one** position and tilt of the square that puts the corners exactly there.

**Real-life example:** a credit card held in front of you. From how big it looks and how its corners are squashed, you can tell how far away it is and how it is tilted.

Each corner $P_i$ of the tag is moved into the camera frame, then projected onto the picture:

$$
P_{c,i}=R_{ct}\,P_{t,i}+t_{ct},\qquad \tilde p_i\sim K\,[R_{ct}\mid t_{ct}]\,\tilde P_{t,i}
$$

OpenCV's `solvePnP` with `IPPE_SQUARE` finds the $R_{ct}$ (rotation) and $t_{ct}$ (position) that fit the 4 corners best. **$t_{ct}$ is the tag's position in metres, relative to the camera.**

## 1.3 From camera coordinates to world coordinates

**How it works:** The camera says "the tag is 1 m in front of me". But the camera is tilted along with the drone, so we rotate that answer by the drone's attitude, then add the drone's position:

$$
\boxed{p_{tag}^{W}=p_{cam}^{W}+R_{WB}\,R_{BC}\,p_{tag}^{C}}
$$

| Symbol | Meaning |
|---|---|
| $p_{tag}^{C}$ | tag position seen by the camera |
| $R_{BC}$ | camera → drone body: how the camera is mounted, $C=\mathrm{diag}(1,-1,-1)$ |
| $R_{WB}=R(q)$ | drone body → world, from the drone's **quaternion** $q$ (Section 4) |
| $p_{cam}^{W}$ | where the camera is in the world |

## 1.4 Tag position → pad centre

**How it works:** The big tag is **not** in the middle of the pad. It sits **0.40 m to one side**. If the drone aimed at the tag, it would land 40 cm off-centre. So we step back from the tag to the centre, and that step **turns with the pad's heading** $\psi$.

![pad centre](md_images/03_pad_centre.png)


$$
\boxed{c_{pad}=p_{tag}-R_z(\psi_{pad})\begin{bmatrix}0.40\\0\end{bmatrix}}
\qquad\Rightarrow\qquad
\begin{aligned}
x_{pad}&=x_{tag}-0.40\cos\psi\\
y_{pad}&=y_{tag}-0.40\sin\psi
\end{aligned}
$$

**Toy example.** For a pad turned by $\psi=30^\circ$, the centre is $0.40\cos 30^\circ = 0.346$ m back in x and $0.40\sin 30^\circ = 0.200$ m back in y. A 2° error in $\psi$ moves the centre by $0.40\times\sin 2^\circ\approx 1.4$ cm. That is why we smooth $\psi$ with SLERP (Section 4.6).

## 1.5 How high can each tag be read?

**How it works:** The detector needs the tag to be **at least 20 pixels wide** in the picture. A tag of size $s$ seen from height $h$ is about $f s/h$ pixels wide.

**Real-life example:** a shop's **big signboard** can be read from across the road, but the **small name plate** on the door only from close up.

$$
\text{pixels}\approx\frac{f\,s}{h}\ \ge\ 20
\quad\Rightarrow\quad
\boxed{h_{max}=\frac{f\,s}{20}}
$$

| Tag | Size $s$ | $h_{max}$ |
|---|---|---|
| big | 0.42 m | $415.7\times0.42/20=$ **8.73 m** |
| small | 0.12 m | $415.7\times0.12/20=$ **2.49 m** |

![tag size vs height](md_images/02_tag_size_vs_height.png)

## 1.6 Why two tags, and the "commit" height

**How it works:**
- The **big tag** can be read from high up. Close to the ground it **no longer fits in the picture**, because it sits 0.40 m off-centre.
- The **small tag** fits close to the ground but **can't be read from high up**.
- Between **1.06 m and 2.49 m both work**, so the drone switches tags there: the **hand-over window**.

**Where 1.06 m comes from.** At height $h$ the camera sees $h\tan 30^\circ=0.577h$ to each side. The far edge of the big tag is $0.40+0.21=0.61$ m from the centre:

$$
0.577\,h\ \ge\ 0.61\ \Rightarrow\ h\ \ge\ 1.06\ \text{m}
$$

**The commit (blind) height.** The same idea is applied to the small tag. Three things change:
- The tag needs a white border: $1.25\times 0.12/2 = 0.075$ m to each side.
- The camera hangs 0.12 m below the drone.
- $e$ is how far the drone is off-centre.

$$
\boxed{h_{blind}=0.12+\frac{0.075+e}{\tan30^\circ}=0.25+1.73\,e}
$$

Below this height **even the small tag doesn't fit in the picture**. Losing the tag there is *expected*, so the drone **commits and lands** instead of climbing back up. Our tests with the real detector measured $0.23+1.55e$, so the formula is slightly on the safe side.

> **Real-life example:** parking a car. In the last few centimetres you can't see the bumper any more. You trust your aim and finish parking instead of driving away.

![pad design](figures/mj_diag_pad.png)

---

# 2. Predicting a moving pad: α-β-γ tracking and turning

## 2.1 Why we need a tracker

**How it works:** The camera only gives a new position 25 times a second, and it gives none in the last 25 cm (blind height). A pad on a van keeps moving in between, so the drone must **estimate where the pad is and where it is going**.

**Real-life example:** a cricket fielder doesn't run to where the ball **is**. They watch it for a moment, judge its speed and curve, and run to where it **will land**.

The tracker keeps 3 numbers for the pad: position $\hat p$, velocity $\hat v$ and acceleration $\hat a$.

**Rule:** *predict* at every control step (100 Hz). *Correct* only when a **new** camera picture arrives (25 Hz).

## 2.2–2.4 Predict → compare → correct

![predict correct](md_images/04_predict_correct.png)

**Step 1: predict** where the pad should be now:

$$
p^-_k=\hat p_{k-1}+\hat v_{k-1}\Delta t+\tfrac12\hat a_{k-1}\Delta t^2,\qquad v^-_k=\hat v_{k-1}+\hat a_{k-1}\Delta t
$$

**Step 2: compare** with what the camera saw. The difference is the "surprise" (residual):

$$
\boxed{r_k=z_k-p^-_k}
$$

**Step 3: correct** each estimate by a fixed share of the surprise:

$$
\hat p_k=p^-_k+\alpha r_k,\qquad
\hat v_k=v^-_k+\frac{\beta}{\Delta t}r_k,\qquad
\hat a_k=\hat a_{k-1}+\frac{2\gamma}{\Delta t^2}r_k
$$

with $\alpha=0.35$, $\beta=0.08$ and $\gamma=0.010$, exactly as in our code.

**Toy example (the picture above).**
- The old estimate is 2 m, moving at 2 m/s, so after 1 s the prediction is $p^-=4.00$ m.
- The camera says $z=5.00$ m, so the surprise is $r=1.00$ m.
- The new estimate is $4.00+0.35\times1.00=$ **4.35 m**. We trust 35% of the surprise, enough to follow the pad without jumping at every noisy pixel.

| Gain | Bigger value means… | Smaller value means… |
|---|---|---|
| $\alpha,\beta,\gamma$ | reacts faster, but noisier | smoother, but slower to react |

## 2.5–2.6 Is the pad turning? The turn rate

**In simple words:**
- Acceleration **along** the velocity means the pad is **speeding up or braking**.
- Acceleration **sideways** to the velocity means the pad is **turning**.

The sideways part tells us how fast it turns.

$$
\boxed{\Omega=\frac{(\mathbf v\times\mathbf a)_z}{\|\mathbf v\|^2}=\frac{v_xa_y-v_ya_x}{v_x^2+v_y^2}}
$$

**Toy example: address 5, a rover driving a 0.9 m circle.**

$$
\mathbf v=(0.54,\,0),\ \ \mathbf a=(0,\,0.324)\ \Rightarrow\
\Omega=\frac{0.54\times0.324}{0.54^2}=0.60\ \text{rad/s},\qquad
\text{radius}=\frac{|v|}{\Omega}=0.9\ \text{m}\ ✔
$$

![turn rate](md_images/05_turn_rate.png)

**Safety gate:** we only believe a turn when two things are true. Otherwise noise could invent a turn that isn't there.

$$
\|\mathbf v\|>0.30\ \text{m/s}\quad\text{and}\quad
\rho=\frac{\|\mathbf v\times\mathbf a\|}{\|\mathbf v\|\|\mathbf a\|}=|\sin\theta|>0.55
$$

## 2.7 Predicting along the curve

**How it works:** If the pad is turning, the drone predicts **along the arc**, not along a straight line. Over the lead time $\tau$ the velocity **rotates** by $\Omega\tau$:

$$
\begin{bmatrix}v_x'\\v_y'\end{bmatrix}=
\begin{bmatrix}\cos\Omega\tau&-\sin\Omega\tau\\ \sin\Omega\tau&\cos\Omega\tau\end{bmatrix}
\begin{bmatrix}v_x\\v_y\end{bmatrix}
$$

**Toy example.** With $\Omega=0.60$ rad/s and a lead of $\tau=0.25$ s, the velocity turns by $0.15$ rad ≈ **8.6°**. In the picture above, after 2.5 s the straight-line guess is about **0.9 m** off, while the arc guess is right on the pad.

![moving landing](figures/mj_fig3_dynamic.png)
*A real run: landing on the moving courier van (address 3).*

---

# 3. Deciding where to fly: PID guidance and feed-forward

## 3.1 The error

$$
\boxed{e=p_{pad}-p_{drone}}\qquad\text{(metres, horizontal, }e=[e_x,\,e_y])
$$

A **positive error** means the pad is ahead of the drone in that direction.

## 3.2 The base paper's PID

| Term | Looks at | Car example | Gain |
|---|---|---|---|
| **P** | how far away the pad is **now** | far from the parking spot → turn the wheel a lot | $K_p=0.20$ |
| **I** | error that **keeps staying** | a crosswind pushes you → hold a small steady correction | $K_i=0.03$ |
| **D** | how fast the error is **shrinking** | closing in fast → ease off | $K_d=0.35$ |

**The paper's Eq. (9)** (gains tuned by experiment on their real DJI M100):

$$
v_{PID}=K_p e+K_i\!\int\! e\,dt+K_d(e-e_{last})
$$

**Our version** divides the change by the time step $\Delta t$:

$$
\dot e_k=\frac{e_k-e_{k-1}}{\Delta t},\qquad I_k=I_{k-1}+e_k\Delta t,\qquad
v_{PID,k}=K_pe_k+K_iI_k+K_d\dot e_k
$$

**Toy example: why "÷ Δt" matters.** The error goes from 1.00 m to 0.98 m in $\Delta t=0.04$ s:

![dt matters](md_images/08_dt_matters.png)

- **With ÷Δt:** $\dot e=-0.02/0.04=-0.5$ m/s, so the D term is $0.35\times(-0.5)=-0.175$ m/s. That is a real brake.
- **Without it (paper as printed):** the D term is $0.35\times(-0.02)=-0.007$ m/s, almost no brake. The strength would also change if the camera's frame rate changed.

**Toy example: static pad 3 m away.** The gap shrinks smoothly, with no overshoot. The time constant is $\tau=(1+K_d)/K_p=1.35/0.20=6.75$ s.

![pid static](md_images/06_pid_static.png)

## 3.3 Why PID alone lags behind a moving pad, and how feed-forward fixes it

**How it works:** A PID only produces speed **from an error**. To follow a van at 0.9 m/s, it must *keep* an error of

$$
e=\frac{V_{pad}}{K_p}=\frac{0.9}{0.20}=\boxed{4.5\ \text{m behind}}
$$

The pad is only 1.5 m wide, so the drone never lands. In our tests the paper's PID landed on **0 of 6** moving pads.

**Feed-forward:** we already know the pad's speed from the tracker, so we **add it directly**:

$$
\boxed{v_{cmd}=v_{PID}+\hat v_{pad}}
\qquad\text{(and on a straight road: }+\ \tau_L\hat a_{pad},\ \tau_L=0.25\text{ s)}
$$

Now $\hat v_{pad}$ supplies the 0.9 m/s, so the PID only has to fix the small leftover, and the **lag becomes ≈ 0**.

> **Real-life example:** throwing a ball to a friend who is running. You aim **where they will be**, by adding their running speed into your throw.

![feed-forward](md_images/07_feedforward.png)

## 3.4 Altitude gain schedule

**How it works:** Near the ground the camera sees only a small patch (at 0.5 m, just ±0.29 m), so a small drift loses the tag. The drone reacts **gently when high** and **sharply when low**. Height $h$ is measured **above the pad's own surface**, so a roof counts from the roof, not from the street.

$$
g(h)=\begin{cases}1,& h\ge 4\ \text{m}\\[2pt] 1+2\,\dfrac{4-h}{3.4},& 0.6<h<4\\[4pt] 3,& h\le0.6\ \text{m}\end{cases}
\qquad\Rightarrow\qquad g(2\text{ m})=2.18
$$

## 3.5 Integral wind-up and our "leaky" integral

**In simple words:** the I term keeps adding up the error. If it collects a big push on the way in, that push can **keep pushing the wrong way** after the drone has arrived. This is called **wind-up**.

**Real-life example:** you lean hard into a strong wind. When the wind suddenly stops, you stumble forward.

**Our fix:**
- The integral only collects error when $|e|<0.6$ m, and it is capped at ±1.2 m·s.
- If the stored integral pushes **against** the current error ($I\cdot e<0$), it drains with a 0.6 s time constant:

$$
I\leftarrow I\left(1-\frac{\Delta t}{0.6}\right)
$$

![windup](md_images/10_windup.png)

On static pads this alone cut the error from **10.9 cm to 2.3 cm**.

## 3.6 Safety cone and commit

**In simple words:** the drone may only go down while it is **well lined up**, and "well lined up" gets stricter as it gets lower, like a **funnel**.

$$
\boxed{\|e\|<0.22\,h}\qquad h=4\ \text{m}\to0.88\ \text{m},\quad h=2\ \text{m}\to0.44\ \text{m},\quad h=0.5\ \text{m}\to0.11\ \text{m}
$$

![schedule and cone](md_images/09_schedule_cone.png)

## 3.7 Our complete guidance law

$$
\boxed{v_{cmd}=g(h)\big[K_pe+K_d\dot e_{filtered}\big]+K_iI_{leaky}+\hat v_{pad}}
$$

The I term is **not** multiplied by $g(h)$. It stands for a steady push, like wind, and tripling it near the ground would make it the strongest term.

| Quantity | Unit | Who makes it |
|---|---|---|
| $e$ | m | camera + tracker |
| $v_{cmd}$ | m/s | **this section** |
| $a_{cmd}$ | m/s² | velocity loop (Section 5) |
| torque $\tau$ | N·m | attitude loop (Section 5) |

### Result: 

| Controller | Moving pads landed | Static-pad error |
|---|---|---|
| Paper PID only | **0 / 6** | 10.9 cm |
| + leaky integral | 0 / 6 | 2.3 cm |
| + **feed-forward** | 4 / 6 | 1.8 cm |
| + gain schedule | **6 / 6** | 0.9 cm |
| + turn tracker (full) | **6 / 6** | **0.6 cm** |

![ablation](figures/mj_fig4_ablation.png)

---

# 4. Finding the pad's pose: Quaternions and SLERP

## 4.1 What a Quaternion is?

**In simple words:** **any** rotation, however complicated, is **one turn about one axis**, like a door turning on its hinge. A quaternion stores that **axis** and **angle** in 4 numbers.

![axis angle and wrap](md_images/11_axis_angle_wrap.png)

$$
\boxed{q=\Big[\cos\tfrac{\theta}{2},\ \hat n\sin\tfrac{\theta}{2}\Big]=[w,x,y,z]},\qquad \boxed{\|q\|=1}
$$

**Toy examples.**

| Rotation | Quaternion |
|---|---|
| no rotation | $[1,0,0,0]$ |
| door opened 90° (vertical axis z) | $[\cos45^\circ,0,0,\sin45^\circ]=[0.707,0,0,0.707]$ |
| drone leaning 10° (axis x) | $[\cos5^\circ,\sin5^\circ,0,0]=[0.996,0.087,0,0]$. Check the length: $0.996^2+0.087^2=1.000$ ✔ |

**Why not Euler angles (roll, pitch, yaw), as in the base paper?**
- **Wrap-around** (right picture above): From 179° to −179° is really **2°**, but subtracting angles gives **358°**. A rover driving in circles crosses ±180° on every lap.
- **Gimbal lock:** Point your arm straight up, and "turn left" and "twist your wrist" become the same movement, so one direction of control is lost. Euler angles do this at 90° pitch.

## 4.2 Conjugate: undo a rotation

$$
\boxed{q^*=[w,-x,-y,-z]},\qquad q^{-1}=q^*\ \ (\text{for }\|q\|=1)
$$

**Real-life example:** the same hinge, turned the other way. For a quaternion that is not unit length, use $q^{-1}=q^*/\|q\|^2$.

## 4.3 Hamilton product: combine two rotations

$$
\begin{aligned}
w&=w_1w_2-x_1x_2-y_1y_2-z_1z_2\\
x&=w_1x_2+x_1w_2+y_1z_2-z_1y_2\\
y&=w_1y_2-x_1z_2+y_1w_2+z_1x_2\\
z&=w_1z_2+x_1y_2-y_1x_2+z_1w_2
\end{aligned}
$$

**Toy example: 45° + 45° = 90°.** With $q=[0.924,0,0,0.383]$:

$$
w=0.924^2-0.383^2=0.707,\qquad z=2\times0.924\times0.383=0.707\ \Rightarrow\ [0.707,0,0,0.707]\ ✔
$$

**Order matters:** $q_1\otimes q_2\ne q_2\otimes q_1$. Tilting your phone forward then sideways ends differently from sideways then forward.

## 4.4 Rotating a vector: the "sandwich"

$$
\boxed{[0,\mathbf p']=q\otimes[0,\mathbf p]\otimes q^*}
$$

**Toy example.** Rotate **east** $(1,0,0)$ by $q=[0.707,0,0,0.707]$:

1. $q\otimes[0,1,0,0]=[0,\,0.707,\,0.707,\,0]$
2. $\otimes\,q^*=[0,\,0,\,1,\,0]$, so the arrow now points **north** ✔

![rotate and slerp](md_images/12_rotate_slerp.png)

## 4.5 $q$ and $-q$ are the same orientation

Turning **+350°** and turning **−10°** end in the same place. That is why a quaternion and its negative describe the same orientation, and we always pick the one that takes the **short way** (shown at work in 5.3).

## 4.6 SLERP: smoothly averaging orientations

**In simple words:** to smooth the pad's heading we blend the old and new headings **along the shortest arc of a circle**, not with a plain average.

**Toy example (right picture above):** averaging 350° and 10° as plain numbers gives $(350+10)/2=$ **180°**, the exact opposite direction ✗. SLERP gives **0°** ✔.

$$
\cos\Omega=q_a\cdot q_b\ (\text{flip }q_b\text{ if negative}),\qquad
\boxed{\mathrm{slerp}(q_a,q_b,t)=\frac{\sin((1-t)\Omega)}{\sin\Omega}q_a+\frac{\sin(t\Omega)}{\sin\Omega}q_b}
$$

**Result:** heading noise fell from **2.7° to 1.6°**. Because of the 0.40 m tag offset (Section 1.4), the pad-centre noise fell from **1.9 cm to 1.1 cm**.

---

# 5. Tilting the drone: desired attitude, error, torque and thrust

## 5.1 Why the drone must tilt

**In simple terms:** the propellers only push **"up" out of the drone's own top**. To move sideways, the drone must **lean**, so part of the push points sideways.

**Real-life example:** a helicopter can't fly sideways while staying flat; it leans towards where it wants to go.

$$
\text{velocity error}\rightarrow a_{cmd}\rightarrow R_{des}\rightarrow q_{des}\rightarrow q_{err}\rightarrow\tau
$$

The velocity loop turns the velocity error into an acceleration: $a_{cmd}=1.6\,(v_{cmd}-v)+1.2\!\int(v_{cmd}-v)\,dt$. The tilt is limited to 25°.

## 5.2 From acceleration to the target attitude $q_{des}$

**Step 1: the force needed**, including holding up the drone's weight:

$$
\boxed{f=m(a_{cmd}+g\,e_3)},\qquad \boxed{b_3=\frac{f}{\|f\|}}\ \ (\text{the new "up" axis})
$$

**Toy example.** To accelerate east at 2 m/s²: $f/m=(2,0,9.81)$, so $b_3=(0.200,0,0.980)$ and the tilt is $\arctan(2/9.81)=$ **11.5°**.

![tilt](md_images/13_tilt.png)

**Step 2: the other two axes.** Pick the heading $c_1$, then use cross products:

$$
b_2=\frac{b_3\times c_1}{\|b_3\times c_1\|},\qquad b_1=b_2\times b_3,\qquad
\boxed{R_{des}=[\,b_1\ b_2\ b_3\,]}\ \rightarrow\ q_{des}
$$

For the toy example, $q_{des}=[\cos5.75^\circ,0,\sin5.75^\circ,0]=[0.995,0,0.100,0]$. No angle is subtracted anywhere, so nothing can wrap around.

## 5.3 The attitude error: one multiplication

$$
\boxed{q_{err}=q^*\otimes q_{des}}\qquad\text{"undo where I am, then go where I want"}
$$

If $w_{err}<0$, flip it: $q_{err}\leftarrow-q_{err}$. This takes the **short way round**.

**Toy example: the 179° → −179° problem solved.**

$$
q=[0.0087,0,0,1.0000],\quad q_{des}=[0.0087,0,0,-1.0000]
$$

$$
q_{err}=[-0.9998,0,0,-0.0175]\ \xrightarrow{\ w<0\ \text{flip}\ }\ [0.9998,0,0,0.0175]
$$

Euler subtraction says **358°** ✗. The quaternion says **2°** ✔.

## 5.4 Error as "axis × angle"

$$
\theta=2\,\mathrm{atan2}(\|\mathbf v\|,w),\qquad
\boxed{e_R=\frac{2\,\mathrm{atan2}(\|\mathbf v\|,w)}{\|\mathbf v\|}\,\mathbf v}
$$

**Toy example.** The drone leans 10° and should be level. Then $q_{err}=[0.996,-0.087,0,0]$, so $e_R=(-0.1745,0,0)$ rad, which means "**turn 10° back**" ✔.

## 5.5 Torque: push towards the target, damp the spin

$$
\boxed{\tau=J(K_R\,e_R-K_\omega\,\omega)}\qquad K_R=130,\ K_\omega=22\ \ (\text{code: }J\cdot10\cdot(13e_R-2.2\omega)\text{, roll and pitch; yaw uses }30,\,12)
$$

This works like P and D again: $K_Re_R$ turns the drone towards the target, and $-K_\omega\omega$ stops it from over-rotating.

**Toy example.** For a small turn, $\ddot\phi+22\dot\phi+130\phi=130\phi_d$. That gives $\omega_n=\sqrt{130}=11.4$ rad/s and $\zeta=22/(2\times11.4)=$ **0.97**: almost no overshoot, settled in about 0.3 s.

![attitude step](md_images/14_attitude_step.png)

## 5.6 Thrust: only the part along the current axis

$$
\boxed{T=f\cdot b_3^{now}},\qquad b_3^{now}=R(q)\,e_3
$$

The drone hasn't finished turning yet, so only the part of $f$ along its **current** "up" axis is useful. Using the full $|f|$ would give extra lift, and the drone would **climb while turning**. Fixing this kept the height change during a 6 m sideways move under 3 cm.

![thrust projection](md_images/15_thrust_projection.png)

## 5.7 How the simulator moves the drone

$$
\dot p=v,\qquad m\dot v=T\,b_3-mg\,e_3+c_d(w-v),\qquad
\boxed{\dot q=\tfrac12\,q\otimes[0,\omega]},\qquad J\dot\omega=\tau-\omega\times J\omegas
$$

| Term | Meaning |
|---|---|
| $c_d(w-v)$ | air drag relative to the wind $w$, with $c_d=0.28$ N per m/s |
| $\dot q=\tfrac12q\otimes[0,\omega]$ | how the orientation changes when spinning at $\omega$ |
| $\omega\times J\omega$ | gyroscopic term (MuJoCo's rigid-body physics includes it) |

After every step the quaternion is renormalised, $q\leftarrow q/\|q\|$, so rounding errors can't stretch it.

**Toy example.** Level drone spinning about z at 1 rad/s, with $\Delta t=0.01$ s:
- $q=[1,0,0,0.005]\to$ normalised $[0.99999,0,0,0.005]$
- the angle turned is $2\arcsin(0.005)=0.57^\circ=1\ \text{rad/s}\times0.01\ \text{s}$ ✔

---

# 6. Conclusion

This project explores the mathematical principles behind vision-guided drone landing using camera geometry, AprilTag detection, PID guidance, moving-pad tracking, pose estimation, quaternion mathematics, and attitude control. These components work together to estimate the landing pad’s position, predict its movement, calculate the required control commands, and adjust the drone’s orientation. The reported simulation results demonstrate improved landing performance through anti-windup, velocity feed-forward, altitude gain scheduling, and coordinated-turn tracking. However, challenging turning trajectories and visual information loss near touchdown remain limitations. Future improvements could include more robust motion estimation and an interacting multiple model (IMM) filter to improve tracking under different movement patterns.

**Results:** **120 / 120** landings across MuJoCo, MATLAB and NetLogo.

| Platform | Mean error | Static pads | Moving pads |
|---|---|---|---|
| MuJoCo | 2.41 cm | 1.21 cm | 5.18 cm |
| MATLAB | 3.38 cm | 1.85 cm | 6.94 cm |
| NetLogo | 2.80 cm | 1.56 cm | 5.69 cm |


