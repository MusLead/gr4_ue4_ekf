import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.animation import FuncAnimation
import time
import os


# === File paths ===
ODOM_FILE = 'odometry.txt'
IMU_FILE = 'imu.txt'

# === Read data from file ===
def read_data(filename):
    try:
        with open(filename, 'r') as f:
            return [list(map(float, line.strip().split())) for line in f if line.strip()]
    except FileNotFoundError:
        return []

# === Global state ===
odometry_data = read_data(ODOM_FILE)
imu_data = read_data(IMU_FILE)
frame_index = 1
max_len = min(len(odometry_data), len(imu_data))

# === Plot limits setup ===
def compute_limits(data1, data2):
    all_x = [x for x, _ in data1 + data2]
    all_y = [y for _, y in data1 + data2]
    return (min(all_x) - 1, max(all_x) + 1, min(all_y) - 1, max(all_y) + 1) if all_x and all_y else (-10, 10, -10, 10)

x_min, x_max, y_min, y_max = compute_limits(odometry_data, imu_data)

# === Figure and Subplots ===
fig, (ax_odom, ax_imu, ax_comb) = plt.subplots(3, 1, figsize=(8, 10), sharex=True, sharey=True)
plt.subplots_adjust(bottom=0.2, hspace=0.4)

# === Odometry Plot ===
odom_line, = ax_odom.plot([], [], 'bo-', label='Odometry')
odom_start, = ax_odom.plot([], [], 'go', label='Start')
odom_end, = ax_odom.plot([], [], 'saddlebrown', marker='o', label='End')
ax_odom.set_title('Odometry')
ax_odom.legend()

# === IMU Plot ===
imu_line, = ax_imu.plot([], [], 'ro-', label='IMU')
imu_start, = ax_imu.plot([], [], 'go', label='Start')
imu_end, = ax_imu.plot([], [], 'saddlebrown', marker='o', label='End')
ax_imu.set_title('IMU')
ax_imu.legend()

# === Combined Plot ===
odom_comb_line, = ax_comb.plot([], [], 'bo-', label='Odometry')
imu_comb_line, = ax_comb.plot([], [], 'ro-', label='IMU')
odom_comb_start, = ax_comb.plot([], [], 'go', label='Odo Start')
imu_comb_start, = ax_comb.plot([], [], 'g^', label='IMU Start')
odom_comb_end, = ax_comb.plot([], [], 'saddlebrown', marker='o', label='Odo End')
imu_comb_end, = ax_comb.plot([], [], 'sienna', marker='^', label='IMU End')
ax_comb.set_title('Combined')
ax_comb.legend()

# Set axes limits
for ax in (ax_odom, ax_imu, ax_comb):
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

# === Slider ===
slider_ax = plt.axes([0.2, 0.05, 0.6, 0.03])
frame_slider = Slider(slider_ax, 'Frame', 1, max_len, valinit=1, valstep=1)

# === Update plot data for a given index ===
def update_plots(idx):
    if len(odometry_data) < idx or len(imu_data) < idx:
        return

    odo = odometry_data[:idx]
    imu = imu_data[:idx]

    if odo:
        x_odo, y_odo = zip(*odo)
        odom_line.set_data(x_odo, y_odo)
        odom_start.set_data(*odo[0])
        odom_end.set_data(*odo[-1])
        odom_comb_line.set_data(x_odo, y_odo)
        odom_comb_start.set_data(*odo[0])
        odom_comb_end.set_data(*odo[-1])

    if imu:
        x_imu, y_imu = zip(*imu)
        imu_line.set_data(x_imu, y_imu)
        imu_start.set_data(*imu[0])
        imu_end.set_data(*imu[-1])
        imu_comb_line.set_data(x_imu, y_imu)
        imu_comb_start.set_data(*imu[0])
        imu_comb_end.set_data(*imu[-1])

    # === Recompute and apply dynamic limits ===
    combined_data = odo + imu
    all_x = [x for x, _ in combined_data]
    all_y = [y for _, y in combined_data]

    if all_x and all_y:
        x_min, x_max = min(all_x) - 1, max(all_x) + 1
        y_min, y_max = min(all_y) - 1, max(all_y) + 1
        for ax in (ax_odom, ax_imu, ax_comb):
            ax.set_xlim(x_min, x_max)
            ax.set_ylim(y_min, y_max)

    fig.canvas.draw_idle()

# === Slider change callback ===
def slider_changed(val):
    update_plots(int(val))

frame_slider.on_changed(slider_changed)

# === Live Update Setup ===
last_update = time.time()
update_timeout = 3  # seconds
last_odom_size = len(odometry_data)
last_imu_size = len(imu_data)

def live_update(frame):
    global odometry_data, imu_data, last_update, last_odom_size, last_imu_size, max_len

    # Read new data
    new_odom = read_data(ODOM_FILE)
    new_imu = read_data(IMU_FILE)

    if len(new_odom) > last_odom_size or len(new_imu) > last_imu_size:
        last_update = time.time()
        odometry_data = new_odom
        imu_data = new_imu
        last_odom_size = len(new_odom)
        last_imu_size = len(new_imu)
        max_len = min(len(odometry_data), len(imu_data))
        frame_slider.valmax = max_len
        frame_slider.ax.set_xlim(frame_slider.valmin, frame_slider.valmax)

    # Stop updating if no new data
    if time.time() - last_update > update_timeout:
        return

    # Automatically move slider to latest frame
    current_idx = int(frame_slider.val)
    if current_idx < max_len:
        frame_slider.set_val(max_len)

# === Animation Start ===
ani = FuncAnimation(fig, live_update, interval=500)

# === Initial plot ===
fig.canvas.manager.set_window_title("Aufgabe B3.1")
update_plots(1)
plt.show()