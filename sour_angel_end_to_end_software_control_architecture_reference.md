# Sour Angel — End-to-End Software & Control Architecture Reference
**Competition:** Techfest 2026–27 — The Wrecking Ball Challenge (IIT Bombay)  
**Team:** Sour Angel (`TM-CCA36657F85`) | **Platform:** $\varnothing 716.41\text{ mm}$ ($2.35\text{ ft}$) Spherical Robot ($M = 30.99\text{ kg}$, $R = 0.3582\text{ m}$)

---

## 1. System Compute & Control Split

To meet the strict $100\%$ onboard autonomy rule within the **NVIDIA Jetson Nano 2GB** memory ceiling (headless JetPack 4.6 / Ubuntu, $4\text{ GB}$ swap), high-rate deterministic stabilization is isolated on the **Raspberry Pi Pico**, while perception, state estimation, and mission logic run on **ROS 2** using Python (`rclpy`) glue nodes paired with compiled C++/CUDA standard packages.

```text
+-----------------------------------------------------------------------------------+
| NVIDIA JETSON NANO 2GB (Headless JetPack 4.6 · ROS 2)                             |
|                                                                                   |
|  +----------------------+   +-----------------------+                             |
|  | v4l2_camera (x2)     |-->| image_proc (x2) [C++] |                             |
|  | (Standard Package)   |   | (Standard Package)    |                             |
|  +----------------------+   +-----------+-----------+                             |
|                                         | /cam_{left,right}/image_rect            |
|                    +--------------------+--------------------+                    |
|                    v                                         v                    |
|  +-----------------------------------+   +-----------------------------------+    |
|  | vision_lane_flow_node.py (rclpy)  |   | yolo_trt_detector_node.py (rclpy) |    |
|  | - IMU pitch de-rotation (OpenCV)  |   | - Ultralytics YOLOv8n TensorRT    |    |
|  | - HSV yellow/red cone IPM line fit|   | - Publishes vision_msgs/          |    |
|  | - Lucas-Kanade (KLT) ground flow  |   |   Detection2DArray (~10 Hz/cam)   |    |
|  +-----------------+-----------------+   +-----------------+-----------------+    |
|                    | /vision/lane_pose & /vision/flow_twist| /vision/detections   |
|                    v                                       |                      |
|  +-----------------------------------+                     |                      |
|  | robot_localization::ekf_node [C++]|                     |                      |
|  | (Standard Package · 100 Hz EKF)   |                     |                      |
|  | - Fuses Pico Odom + IMU + Vision  |                     |                      |
|  +-----------------+-----------------+                     |                      |
|                    | /odometry/filtered                    |                      |
|                    v                                       v                      |
|  +-----------------------------------------------------------------------------+  |
|  | mission_guidance_node.py (rclpy · 50-100 Hz)                                |  |
|  | - Sector 1-4 State Machine + Chevron Distance Reset                         |  |
|  | - Sector 4 Pendulum Bob Sinusoid Fitter y(t) = A*sin(2*pi*t/T + phi_0)      |  |
|  | - Lateral Guidance Law: phi_ref = -clip(0.15*e_y + 0.6*psi_e, -15, +15 deg) |  |
|  +-------------------------------------+---------------------------------------+  |
|                                        | sour_angel_interfaces/PicoSetpoint       |
|                                        v                                          |
|  +-----------------------------------------------------------------------------+  |
|  | Blitz ROS bridge (upstream package + Sour Angel interfaces.py mapping)       |  |
|  | - TX (100 Hz): PicoSetpoint -> packet ID 10                                 |  |
|  | - RX (200 Hz): packet ID 11 -> /pico/telemetry (PicoTelemetry)              |  |
|  +-------------------------------------+---------------------------------------+  |
|                                        | PicoTelemetry                          |
|                                        v                                          |
|  +-----------------------------------------------------------------------------+  |
|  | pico_state_bridge_node.py (rclpy)                                            |  |
|  | - PicoTelemetry -> standard IMU, odometry, diagnostics, and flat status     |  |
|  +-------------------------------------+---------------------------------------+  |
+----------------------------------------+------------------------------------------+
                                         | USB CDC Serial (Blitz library uses Arduino Serial)
+----------------------------------------+------------------------------------------+
| RASPBERRY PI PICO (C++ Arduino-Pico · PlatformIO · Dual-Core Real-Time)            |
|                                        v                                          |
|  +--------------------------+    +---------------------------------------------+  |
|  | Core 1: Blitz & Safety   |--->| Core 0: 400 Hz Stabilization Loop           |  |
|  | [BLITZ USB CDC]          |    | - BNO085 (SPI) + PIO Encoders + AS5600      |  |
|  | - 50 Hz Kill-Switch WDT  |    | - Longitudinal & Lateral LQR                |  |
|  | - 200 ms Jetson WDT      |    | - 0.15 Hz Roll Allocator (Wheel/Pendulum)   |  |
|  +--------------------------+    +---------------------------------------------+  |
+-----------------------------------------------------------------------------------+
```

---

## 2. Blitz Serial Interface

The MCU implementation uses the Arduino-Pico core and PlatformIO. The checked-in Blitz MCU library owns framing and serialization; Sour Angel packet IDs and payload types are in `firmware/pico/lib/Blitz/blitz_interfaces.hpp`. The transport adapter remains isolated from the controller and ROS-facing messages. The current Blitz wire format is sync byte `0xAA`, packet ID, then a fixed-size payload. The library accesses Arduino `Serial`, so the Pico host link is USB CDC. Its framing has no packet checksum; adding one requires a compatible, versioned change on both Pico and Jetson. Validate lengths, finite floats, sequence ordering, and packet IDs at the application boundary.

### 2.1 Telemetry Frame (`Pico -> Jetson` @ $200\text{ Hz}$)
* **`timestamp_us` (`uint32`):** Pico microsecond clock for latency alignment.
* **`theta`, `theta_dot` (`float32`):** Internal chassis pitch ($\text{rad}$) and pitch rate ($\text{rad/s}$).
* **`phi`, `phi_dot` (`float32`):** Chassis roll ($\text{rad}$) and roll rate ($\text{rad/s}$).
* **`gyro_z` (`float32`):** Calibrated `BNO085` yaw angular velocity ($\text{rad/s}$).
* **`enc_vel_l`, `enc_vel_r` (`float32`):** Relative angular velocities ($\text{rad/s}$) of left/right side caps from `RS775` Hall encoders.
* **`pend_angle` (`float32`):** Lateral pendulum swing angle ($\text{rad}$) from `PG45` encoder.
* **`flywheel_rpm` (`float32`):** Momentum wheel speed ($\text{rpm}$) from `AS5600`.
* **`pack_voltage` (`float32`):** $6\text{S}$ LiPo voltage ($\text{V}$) from `INA228`.
* **`fault_flags` (`uint8`):** Bitmask (`Bit 0`: HW Kill, `Bit 1`: LVC Warn $\le 21.6\text{ V}$, `Bit 2`: LVC Crit $\le 19.8\text{ V}$, `Bit 3`: Motor Stall).

### 2.2 Setpoint Frame (`Jetson -> Pico` @ $100\text{ Hz}$)
* **`seq` (`uint32`):** Rolling sequence counter.
* **`x_ref` (`float32`):** Target along-track displacement ($\text{m}$).
* **`v_ref` (`float32`):** Target forward velocity ($\text{m/s}$).
* **`phi_ref` (`float32`):** Target roll steering angle ($\text{rad}$, clipped to $\pm 15^\circ$).
* **`estop_flag` (`uint8`):** Software safe-stop command (`1` = immediate zero torque).

### 2.3 Packet IDs and Transport Boundary
* **Jetson (Blitz ROS package):** Uses `ros2/blitz/blitz/interfaces.py` to map flat `PicoSetpoint` and `PicoTelemetry` fields to Blitz packet IDs 10 and 11. The bridge publishes `/pico/telemetry` and subscribes to `/control/cmd_setpoints`.
* **RPi Pico (`setup1` / `loop1`, Core 1):** `BlitzSerialTransport` calls the checked-in Blitz API. Packet ID `10` is the 17-byte setpoint; packet ID `11` is the 45-byte telemetry frame. It publishes valid setpoints to the mailbox. Core 0 never calls serial APIs.

Use the checked-in Blitz MCU library as the transport implementation; do not duplicate its packet framing or serializer. Its local `receive_data()` returns the packet ID plus the full payload, which matches `parse_struct()`. The Sour Angel adapter checks the expected ID and exact packet size before parsing. The wire protocol has no integrity check, so CRC protection remains a future protocol extension that requires matching host and MCU changes. Keep serial service from starving kill-switch and watchdog supervision.

---

## 3. Jetson Nano ROS 2 Architecture

### 3.1 Standard Off-the-Shelf ROS 2 Packages
| Package | Node / Executable | Rate | Function |
| :--- | :--- | :--- | :--- |
| **`v4l2_camera`** | `v4l2_camera_node` ($\times 2$) | $30\text{ Hz}$ | Captures $1280 \times 720$ MJPEG streams from left/right Logitech axle-pod webcams ($\pm 39^\circ$ FOV, $358\text{ mm}$ height, $\pm 57.49\text{ mm}$ offset). |
| **`image_proc`** | `rectify_node` ($\times 2$) | $30\text{ Hz}$ | C++ lens undistortion publishing `/cam_{left,right}/image_rect`. |
| **`tf2_ros`** | `static_transform_publisher` | Static | Publishes rigid transforms from `base_link` to `cam_left_link` and `cam_right_link`. |
| **`robot_localization`** | `ekf_node` | $100\text{ Hz}$ | C++ EKF fusing wheel odometry, IMU yaw rate, lane pose, and optical flow into `/odometry/filtered`. |
| **`rosbag2`** | `ros2 bag record` | Async | Records `/pico/imu`, `/pico/wheel_odom`, `/odometry/filtered`, `/vision/detections`, and `/mission/state`. |

### 3.2 Custom Python (`rclpy`) Nodes
1. **Blitz ROS bridge ($200\text{ Hz}$ RX / $100\text{ Hz}$ TX):**
   * Uses the mapping in `ros2/blitz/blitz/interfaces.py`. Flat `PicoSetpoint` fields come from `/control/cmd_setpoints`; raw `PicoTelemetry` is published to `/pico/telemetry`.
2. **`pico_state_bridge_node.py` ($200\text{ Hz}$):**
   * Converts `/pico/telemetry` into standard `/pico/imu` (`sensor_msgs/Imu`), `/pico/wheel_odom` (`nav_msgs/Odometry`), and `/pico/diagnostics` (`diagnostic_msgs/DiagnosticArray`). It creates `PicoStatus` from telemetry fault bits and bridge-side command age/transport state.
   * **Critical Pendulum Kinematic Correction:** Because the `RS775` encoders measure relative rotation between the hanging internal chassis and the outer shell, this node must add chassis pitch rate before computing ground velocity:
     $$\omega_{\text{shell}} = \frac{\omega_{\text{enc,L}} + \omega_{\text{enc,R}}}{2} + \dot{\theta}_{\text{IMU}}, \qquad v_{\text{odom}} = R \cdot \omega_{\text{shell}}$$
   * Publishes `/pico/imu`, `/pico/wheel_odom`, `/pico/diagnostics`, and `/pico/status` (`sour_angel_interfaces/PicoStatus`).
3. **`vision_lane_flow_node.py` ($30\text{ Hz}$):**
   * Uses `cv_bridge` and `image_geometry.PinholeCameraModel`.
   * **Pitch De-Rotation:** Rotates each rectified frame backward by $-\theta_{\text{IMU}}$ (`cv2.warpAffine`) because the camera optical axis lies along the transverse axle.
   * **Lane Extraction:** Applies HSV masks (`cv2.inRange`) to isolate **yellow cones** (left camera) and **red cones** (right camera), projects cone bases onto the $z = -0.358\text{ m}$ ground plane via Inverse Perspective Mapping (IPM), and fits lines through the nearest $5$ cones per side to publish lateral offset $e_y$ and heading error $\psi_e$ on `/vision/lane_pose` (`geometry_msgs/PoseWithCovarianceStamped`).
   * **Ground Optical Flow:** Runs Lucas-Kanade sparse tracking (`cv2.calcOpticalFlowPyrLK`) on ground ROI patches to publish forward speed $v_{\text{flow}}$ and yaw rate $\dot{\psi}_{\text{flow}}$ on `/vision/flow_twist` (`geometry_msgs/TwistWithCovarianceStamped`).
4. **`yolo_trt_detector_node.py` ($\approx 10\text{ Hz}$ per camera, alternating):**
   * Runs `ultralytics.YOLO` loaded with a TensorRT FP16 engine ($320 \times 320$ input resolution).
   * Publishes `vision_msgs/Detection2DArray` on `/vision/detections` for classes: `cone_yellow`, `cone_red`, `chevron_gate`, `ramp_edge`, `wreck_block`, and `pendulum_bob`.
5. **`mission_guidance_node.py` ($50\text{--}100\text{ Hz}$):**
   * Subscribes to `/odometry/filtered`, `/vision/detections`, and `/pico/diagnostics`; publishes flat `sour_angel_interfaces/PicoSetpoint` on `/control/cmd_setpoints`.
   * Computes lateral roll reference:
     $$\phi_{\text{ref}} = -\text{clip}\!\left(0.15\,e_y + 0.6\,\psi_e,\; -15^\circ,\; +15^\circ\right)$$
   * Manages the 6-state Sector FSM, calls `/set_pose` on Sector 1 chevron landmarks, and runs the Sector 4 pendulum trajectory predictor.

### 3.3 `robot_localization` (`ekf_node`) Configuration Rules for Pendulum Kinematics
Because `ekf_node` assumes a rigid body frame while the internal chassis pitches up to $65^\circ$ and rolls $\pm 15^\circ$, enforce these exact settings in `config/ekf.yaml`:
* **`two_d_mode: true`:** Constrains state estimation to planar track coordinates $(s, e_y, \psi_e, v, \dot{\psi})$, isolating internal pendulum tilt.
* **IMU Fusion Matrix:** Fuse **only yaw angular velocity ($\dot{\psi}$)** from `/pico/imu`. Set linear accelerations ($\ddot{x}, \ddot{y}$) and roll/pitch angles ($\phi, \theta$) to `false` so gravity projections during pendulum tilt do not corrupt position estimates.
* **Latency & Outlier Handling:** Enable `smooth_lagged_data: true` with `history_length: 0.2` to rewind and fuse $\approx 100\text{ ms}$-delayed webcam updates. Set Mahalanobis rejection thresholds to `3.03` ($\approx 99\%$ chi-square gate).
* **Slip Mitigation:** Assign lower covariance (higher trust) to `/vision/flow_twist` ($v_x$) than to `/pico/wheel_odom` ($v_x$) so KLT ground flow overrides wheel slip during hard launches and Sector 3 obstacle impacts.

---

## 4. Raspberry Pi Pico Arduino-Pico $400\text{ Hz}$ Real-Time Control Architecture

### 4.1 Dual-Core Task Partitioning
* **Core 1 (`setup1` / `loop1`, Serial & Watchdog Supervisor):**
  * Services `blitz_pico_try_receive()` ($100\text{ Hz}$) and `blitz_pico_send_telemetry()` ($200\text{ Hz}$).
  * Monitors the wireless kill-switch receiver ($50\text{ Hz}$ coded heartbeat; opens the $80\text{ A}$ motor relay within $< 150\text{ ms}$ if lost for $> 100\text{ ms}$).
* **Core 0 (`setup` / `loop`, $400\text{ Hz}$ / $2.5\text{ ms}$ control loop):**
  1. **Sensor Polling:** Reads `BNO085` IMU via SPI at $400\text{ Hz}$ (Game Rotation Vector + calibrated gyro; magnetometer disabled due to motor magnetic interference), samples $3\times$ hardware PIO quadrature decoders (`2x RS775` Hall encoders + `PG45` encoder), and reads `AS5600` (wheel speed) and `INA228` (pack voltage) over I2C.
  2. **Longitudinal LQR (Propulsion):**
     * Operates on longitudinal error state $[\tilde{x}, \dot{\tilde{x}}, \theta, \dot{\theta}]^T$ using gain vector:
       $$K_{\text{long}} = \begin{bmatrix} 28.0 & 97.1 & 107.7 & 22.4 \end{bmatrix}$$
     * Computes $\tau_d = -K_{\text{long}}[x-x_{ref},\;v-v_{ref},\;\theta,\;\dot{\theta}]^T$ and clamps drive torque to $\pm 13.9\text{ N}\cdot\text{m}$ (baseline CoG $d = 50.41\text{ mm}$ at $65^\circ$ pendulum stop; expandable to $40.6\text{ N}\cdot\text{m}$ if CoG is lowered to $d \ge 147\text{ mm}$). `LongitudinalController` implements this calculation; the motor layer drives `2x RS775` (`38.4:1` ratio) via dual `BTS7960` H-bridges with a $12\text{ A/motor}$ current cap and $2\text{ s}$ stall timeout.
  3. **Lateral LQR & Hybrid Roll Allocation ($0.15\text{ Hz}$ Crossover):**
     * Computes total roll torque $\tau_{\text{roll}} = -K_{\text{lat}} [\phi - \phi_{\text{ref}},\, \dot{\phi}]^T$ with:
       $$K_{\text{lat}} = \begin{bmatrix} 113.5 & 44.0 \end{bmatrix}$$
     * **Frequency Split:** A first-order low-pass filter at $0.15\text{ Hz}$ routes slow steering lean ($< 0.15\text{ Hz}$) to the `PG45` lateral pendulum (`1:51` planetary $\times 1.7:1$ spur, `BTS7960` capped at $6\text{ A}$, $\pm 45^\circ$ stops) and fast roll damping ($> 0.15\text{ Hz}$) to the $340\text{ mm}$ BLDC momentum wheel (`270 KV` outrunner). Gear-backlash compensation remains in the actuator layer pending characterization.
     * **Band Overrides & Desaturation:** `LateralController` routes roll error $|\phi_{\text{err}}| < 2^\circ$ to the momentum wheel and engages the pendulum at full rate when $|\phi_{\text{err}}| > 6^\circ$; the band between them uses the $0.15\text{ Hz}$ split. It guards the pendulum's $\pm45^\circ$ stops and wheel's $2000\text{ rpm}$ limit, and raises a desaturation request at $1200\text{ rpm}$. The bias magnitude and sign need actuator characterization before enabling the desaturation action.
     * **ESC Speed Ramp:** Converts wheel torque $\tau_w$ to an ESC speed command via $\frac{d\Omega}{dt} = \frac{\tau_w}{I_w}$, hard-capped at $2000\text{ rpm}$ and commanded to zero throttle (coast) on kill.

---

## 5. Sector-by-Sector Mission State Machine ($75\text{ m}$ Track)

Scoring formula: $\text{FinalScore (s)} = \text{RawRunTime} + 5\text{ s} \times \text{ConeContacts}$.

| State | Track Span | Velocity & Control Policy | Transition / Exit Condition |
| :--- | :--- | :--- | :--- |
| **`S1_GATE_RUNWAY`** | $0\text{ m} \to 15\text{ m}$ | Command $v_{\text{ref}} = 4.0\text{ m/s}$. Detect chevron gates (`YOLOv8n`) to verify camera extrinsics and reset $s$ via `/set_pose`. | $s \ge 15.0\text{ m} \to$ `S2_RAMPS` |
| **`S2_RAMPS`** | $15\text{ m} \to 35\text{ m}$ | Maintain entry speed $\ge 2.5\text{ m/s}$ ($v_{\text{ref}} = 3.0\text{ m/s}$) to carry momentum up the $0.2\text{ ft}$, $0.4\text{ ft}$, and $0.8\text{ ft}$ ramps (predicted exit $\approx 2.0\text{ m/s}$). | $s \ge 35.0\text{ m} \to$ `S3_WRECKING_ZONE` |
| **`S3_WRECKING_ZONE`** | $35\text{ m} \to 55\text{ m}$ | Command $v_{\text{ref}} = 3.0\text{ m/s}$; plow directly through foam/cardboard blocks while prioritizing KLT optical flow over encoders. | `pendulum_bob` detected within $1.2\text{ m} \to$ `S4_STANDOFF_HOLD` |
| **`S4_STANDOFF_HOLD`** | $\approx 55\text{ m}+$ | Halt $0.8\text{ m}$ before the swing plane (full swing visible inside $\pm 39^\circ$ FOV). Fit $y(t) = A\sin(2\pi t/T + \phi_0)$ across two full oscillations. | Predicted clear window $\ge 2.4\text{ s}$ ($1.9\text{ s}$ transit at $0.86\text{ m/s}^2 + 0.5\text{ s}$ margin) $\to$ `S5_SPRINT` |
| **`S5_SPRINT`** | Swing $\to 75\text{ m}$ | Accelerate at max flat-ground ceiling ($0.86\text{ m/s}^2$, $v_{\text{ref}} = 4.2\text{ m/s}$) through swing plane. **Abort back to `S4`** if bob re-enters lane early. | $s \ge 75.0\text{ m} \to$ `S6_FINISH` |
| **`S6_FINISH` / `FAULT`** | $75\text{ m}$ / Any | Command $v_{\text{ref}} = 0$, $\phi_{\text{ref}} = 0$ (or `estop_flag = 1` on critical fault). | Run complete / Safe stop |

---

## 6. Software Failsafe & Watchdog Matrix

| Fault Trigger | Monitor | Autonomous Action |
| :--- | :--- | :--- |
| **Wireless Kill Pressed or Link Lost ($> 100\text{ ms}$)** | Pico `Core 1` ($50\text{ Hz}$ RX) | Hardware relay opens motor bus ($< 150\text{ ms}$), `Core 0` zeroes all torques within $2.5\text{ ms}$, BLDC ESC coasts. |
| **Jetson Hang / Blitz link loss ($> 200\text{ ms}$)** | Pico `Core 0` setpoint freshness check | Pico overrides $v_{\text{ref}} = 0$ and $\phi_{\text{ref}} = 0$ to execute a balanced, controlled stop. |
| **RPi Pico Firmware Hang** | RP2040 Hardware WDT | Resets MCU; external pull-downs pull all `BTS7960` driver enable lines LOW. |
| **Battery Low-Voltage Warning ($\le 21.6\text{ V}$)** | Pico `Core 0` (`INA228`) | Caps longitudinal velocity reference $v_{\text{ref}}$ at $1.5\text{ m/s}$ ($3.6\text{ V/cell}$). |
| **Battery Critical Cutoff ($\le 19.8\text{ V}$)** | Pico `Core 0` (`INA228`) | Zeroes all motor torque ($3.3\text{ V/cell}$) and signals Jetson Nano to initiate clean OS shutdown. |

---

## 7. Phased Implementation & Bring-Up Plan

### Phase 1: PlatformIO Arduino-Pico Firmware & Blitz USB Loopback (Bench Validation)
1. **Sensor & PIO Bring-Up:** Verify `400 Hz` `BNO085` SPI reads, `3x` PIO quadrature encoder polarity, `AS5600` RPM readings, and `INA228` voltage scaling.
2. **Blitz Transport:** Validate $200\text{ Hz}$ telemetry and $100\text{ Hz}$ setpoint exchange with the Blitz ROS bridge over USB CDC; verify the $200\text{ ms}$ Jetson watchdog and $< 150\text{ ms}$ wireless kill cutoff.
3. **Static Cradle Stabilization:** Mount the internal chassis on an axle test stand and tune `K_long`, `K_lat`, the $0.15\text{ Hz}$ crossover filter, and wheel desaturation above $1200\text{ rpm}$.

### Phase 2: Jetson Nano ROS 2 Pipeline & `rosbag2` Replay (Simulation Audit)
1. **Camera & Rectification:** Calibrate both Logitech webcams (`camera_calibration`), launch `v4l2_camera` + `image_proc`, and confirm static TF frames.
2. **Vision & TensorRT Export:** Export `YOLOv8n` to TensorRT FP16 ($320 \times 320$) and verify $\approx 10\text{ Hz/cam}$ inference alongside $30\text{ Hz}$ HSV cone IPM and KLT optical flow.
3. **EKF & FSM Verification:** Tune `ekf.yaml` (`two_d_mode: true`, yaw-rate only IMU, lagged data smoothing) using recorded `rosbag2` telemetry and verify Sector 4 sinusoid phase locking on a mock swinging pendulum.

### Phase 3: Full-Vehicle Track Integration (Pre-Finals Trials)
1. **Straight-Line & Cone-Slalom Trials:** Validate $\omega_{\text{shell}} = \omega_{\text{encoder}} + \dot{\theta}_{\text{IMU}}$ odometry accuracy and lateral guidance gains ($0.15\,e_y + 0.6\,\psi_e$) for zero cone contacts.
2. **Ramp Momentum & Wrecking Zone Runs:** Verify $\ge 2.5\text{ m/s}$ entry speed across the $0.8\text{ ft}$ ramp (or $147\text{ mm}$ lowered CoG climb from rest) and confirm KLT flow prevents drift during foam/cardboard impacts.
3. **End-to-End $75\text{ m}$ Dress Rehearsal:** Execute full autonomous runs including Sector 4 stand-off hold, sinusoid launch, and early-reentry abort.
