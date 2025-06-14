import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.animation import FuncAnimation
import time
import os
import numpy as np
import signal
import sys

# === File paths ===
ODOM_FILE = 'odometry.txt'
ODOM_FILTERED_FILE = 'odometry_filtered.txt'
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
odometry_filtered_data = read_data(ODOM_FILTERED_FILE)
imu_data = read_data(IMU_FILE)
frame_index = 1
max_len = min(len(odometry_data), len(odometry_filtered_data), len(imu_data))

# === Axis limit tracking ===
odom_xlim = odom_ylim = None
odom_filt_xlim = odom_filt_ylim = None
imu_xlim = imu_ylim = None
comb_xlim = comb_ylim = None

# === Signal handler to exit cleanly on Ctrl+C ===
def signal_handler(sig, frame):
    plt.close('all')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# === Figure and Subplots (2x2 layout) ===
fig, axs = plt.subplots(2, 2, figsize=(12, 10))
(ax_odom, ax_odom_filt), (ax_imu, ax_comb) = axs
plt.subplots_adjust(bottom=0.15, hspace=0.4, wspace=0.3)

# === Plot Elements Initialization ===
# Raw Odometry
odom_line, = ax_odom.plot([], [], 'bo-', label='Raw Odometry')
odom_start, = ax_odom.plot([], [], 'go', label='Start')
odom_end, = ax_odom.plot([], [], 'saddlebrown', marker='o', label='End')
ax_odom.set_title('Raw Odometry')
ax_odom.legend()

# Filtered Odometry
odom_filt_line, = ax_odom_filt.plot([], [], 'co-', label='Filtered Odometry')
odom_filt_start, = ax_odom_filt.plot([], [], 'go', label='Start')
odom_filt_end, = ax_odom_filt.plot([], [], 'saddlebrown', marker='o', label='End')
ax_odom_filt.set_title('Filtered Odometry')
ax_odom_filt.legend()

# IMU
imu_line, = ax_imu.plot([], [], 'ro-', label='IMU')
imu_start, = ax_imu.plot([], [], 'go', label='Start')
imu_end, = ax_imu.plot([], [], 'saddlebrown', marker='o', label='End')
ax_imu.set_title('IMU')
ax_imu.legend()

# Combined
odom_comb_line, = ax_comb.plot([], [], 'bo-', label='Raw Odometry')
odom_filt_comb_line, = ax_comb.plot([], [], 'co-', label='Filtered Odometry')
imu_comb_line, = ax_comb.plot([], [], 'ro-', label='IMU')

odom_comb_start, = ax_comb.plot([], [], 'go', label='Raw Start')
odom_filt_comb_start, = ax_comb.plot([], [], 'g^', label='Filt Start')
imu_comb_start, = ax_comb.plot([], [], 'g*', label='IMU Start')

odom_comb_end, = ax_comb.plot([], [], 'saddlebrown', marker='o', label='Raw End')
odom_filt_comb_end, = ax_comb.plot([], [], 'darkcyan', marker='^', label='Filt End')
imu_comb_end, = ax_comb.plot([], [], 'sienna', marker='*', label='IMU End')

ax_comb.set_title('Combined View')
ax_comb.legend()

# === Slider ===
slider_ax = plt.axes([0.2, 0.05, 0.6, 0.03])
frame_slider = Slider(slider_ax, 'Frame', 1, max_len, valinit=1, valstep=1)

# === Helper: Expanding axis limits with adaptive padding ===
def set_expanding_limits(ax, data, prev_xlim, prev_ylim):
    if not data:
        return prev_xlim, prev_ylim

    x_vals, y_vals = zip(*data)
    min_x, max_x = min(x_vals), max(x_vals)
    min_y, max_y = min(y_vals), max(y_vals)

    # Determine the range for x and y separately
    range_x = abs(max_x - min_x)
    range_y = abs(max_y - min_y)

    # Calculate adaptive padding for x-axis
    if range_x <= 1.0:
        padding_x = round(range_x * 0.2, 5)
    elif range_x <= 10:
        padding_x = 0.5
    else:
        magnitude_x = int(np.log10(range_x))
        padding_x = 10 ** magnitude_x + 1

    # Calculate adaptive padding for y-axis
    if range_y <= 1.0:
        padding_y = round(range_y * 0.2, 5)
    elif range_y <= 10:
        padding_y = 0.5
    else:
        magnitude_y = int(np.log10(range_y))
        padding_y = 10 ** magnitude_y + 1

    # Handle edge cases
    if min_x == max_x:
        min_x -= padding_x
        max_x += padding_x
    else:
        min_x -= padding_x
        max_x += padding_x

    if min_y == max_y:
        min_y -= padding_y
        max_y += padding_y
    else:
        min_y -= padding_y
        max_y += padding_y

    new_xlim = (
        min(min_x, prev_xlim[0]) if prev_xlim else min_x,
        max(max_x, prev_xlim[1]) if prev_xlim else max_x
    )
    new_ylim = (
        min(min_y, prev_ylim[0]) if prev_ylim else min_y,
        max(max_y, prev_ylim[1]) if prev_ylim else max_y
    )

    ax.set_xlim(new_xlim)
    ax.set_ylim(new_ylim)
    return new_xlim, new_ylim

# === Update plot data for a given index ===
def update_plots(idx):
    global odom_xlim, odom_ylim, odom_filt_xlim, odom_filt_ylim, imu_xlim, imu_ylim, comb_xlim, comb_ylim

    odo = odometry_data[:idx]
    odo_filt = odometry_filtered_data[:idx]
    imu = imu_data[:idx]

    if odo:
        x, y = zip(*odo)
        odom_line.set_data(x, y)
        odom_start.set_data([odo[0][0]], [odo[0][1]])
        odom_end.set_data([odo[-1][0]], [odo[-1][1]])
        odom_comb_line.set_data(x, y)
        odom_comb_start.set_data([odo[0][0]], [odo[0][1]])
        odom_comb_end.set_data([odo[-1][0]], [odo[-1][1]])

    if odo_filt:
        x, y = zip(*odo_filt)
        odom_filt_line.set_data(x, y)
        odom_filt_start.set_data([odo_filt[0][0]], [odo_filt[0][1]])
        odom_filt_end.set_data([odo_filt[-1][0]], [odo_filt[-1][1]])
        odom_filt_comb_line.set_data(x, y)
        odom_filt_comb_start.set_data([odo_filt[0][0]], [odo_filt[0][1]])
        odom_filt_comb_end.set_data([odo_filt[-1][0]], [odo_filt[-1][1]])

    if imu:
        x, y = zip(*imu)
        imu_line.set_data(x, y)
        imu_start.set_data([imu[0][0]], [imu[0][1]])
        imu_end.set_data([imu[-1][0]], [imu[-1][1]])
        imu_comb_line.set_data(x, y)
        imu_comb_start.set_data([imu[0][0]], [imu[0][1]])
        imu_comb_end.set_data([imu[-1][0]], [imu[-1][1]])

    odom_xlim, odom_ylim = set_expanding_limits(ax_odom, odo, odom_xlim, odom_ylim)
    odom_filt_xlim, odom_filt_ylim = set_expanding_limits(ax_odom_filt, odo_filt, odom_filt_xlim, odom_filt_ylim)
    imu_xlim, imu_ylim = set_expanding_limits(ax_imu, imu, imu_xlim, imu_ylim)
    combined_data = odo + odo_filt + imu
    comb_xlim, comb_ylim = set_expanding_limits(ax_comb, combined_data, comb_xlim, comb_ylim)

    fig.canvas.draw_idle()

# === Slider callback ===
def slider_changed(val):
    update_plots(int(val))

frame_slider.on_changed(slider_changed)

# === Live Update Setup ===
last_update = time.time()
update_timeout = 3  # seconds
last_odom_size = len(odometry_data)
last_odom_filt_size = len(odometry_filtered_data)
last_imu_size = len(imu_data)

def live_update(frame):
    global odometry_data, odometry_filtered_data, imu_data
    global last_update, last_odom_size, last_odom_filt_size, last_imu_size, max_len

    new_odom = read_data(ODOM_FILE)
    new_odom_filt = read_data(ODOM_FILTERED_FILE)
    new_imu = read_data(IMU_FILE)

    if (len(new_odom) > last_odom_size or
        len(new_odom_filt) > last_odom_filt_size or
        len(new_imu) > last_imu_size):

        last_update = time.time()
        odometry_data = new_odom
        odometry_filtered_data = new_odom_filt
        imu_data = new_imu
        last_odom_size = len(new_odom)
        last_odom_filt_size = len(new_odom_filt)
        last_imu_size = len(new_imu)
        max_len = min(len(odometry_data), len(odometry_filtered_data), len(imu_data))
        frame_slider.valmax = max_len
        frame_slider.ax.set_xlim(frame_slider.valmin, frame_slider.valmax)

    if time.time() - last_update > update_timeout:
        return

    current_idx = int(frame_slider.val)
    if current_idx < max_len:
        frame_slider.set_val(max_len)

# === Animation Start ===
ani = FuncAnimation(fig, live_update, interval=500, cache_frame_data=False)

# === Initial plot ===
fig.canvas.manager.set_window_title("Odometry, Filtered & IMU - Centered View")
update_plots(1)
plt.show()
