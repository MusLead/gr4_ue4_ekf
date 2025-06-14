# 🧭 Extended Kalman Filter (EKF) Visualization – `ekf_ue4`

This project provides a basic ROS 2-based EKF (Extended Kalman Filter) setup and a Python-based visualization tool for analyzing robot odometry, filtered EKF output, and IMU data.

---

## 🔧 How to Run This Project

### 📁 Step 1 – Go to Your Workspace

Before running anything, make sure you're inside your ROS 2 workspace:

```bash
cd ~/your_workspace_directory  # e.g., cd ~/ros2_ws or cd ~/ekf_ue4_ws
```

Then, **build the workspace**:

```bash
colcon build
```

Then, **source the workspace**:

```bash
source install/setup.bash
```

---

### 🚀 Step 2 – Launch the EKF Node

To start publishing and filtering data using EKF, run:

```bash
ros2 launch ekf_gr4 ekf_launch.py
```

This will:
- Launch your robot nodes (including odometry and IMU).
- Launch the `robot_localization` package for EKF.
- Start saving output into text files:
  - `odometry.txt`
  - `odometry_filtered.txt`
  - `imu.txt`

---

### 📈 Step 3 – Run the Plotting Tool

Once data is being generated, use the plotting script to visualize it:

```bash
python3 plot_combined.py
```

This will open a GUI window with:
- 4 subplots: Raw Odometry, Filtered Odometry, IMU, and Combined view.
- A slider to select frames interactively.
- Live update support (refreshes when data changes).
- Mouse-based pan and zoom (scroll to zoom, right-click to drag).

---

## 📂 File Overview

| File                  | Description                                 |
|-----------------------|---------------------------------------------|
| `plot_combined.py`    | Visualization script with live and interactive plotting |
| `ekf_launch.py`       | ROS 2 launch file to start EKF and sensors  |
| `odometry.txt`        | Raw odometry data output                    |
| `odometry_filtered.txt` | Filtered output from EKF                  |
| `imu.txt`             | IMU sensor readings                         |

---

## ✅ Requirements

- ROS 2 (e.g., Humble or Foxy)
- [`robot_localization`](https://github.com/cra-ros-pkg/robot_localization) package
- Python 3
- Python packages:
  ```bash
  pip install matplotlib numpy
  ```

---
