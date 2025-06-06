import serial
import matplotlib.pyplot as plt
from collections import deque
import csv

# Serial port settings
ser = serial.Serial('COM6', 115200, timeout=1)  # Replace 'COMx' with your port

# Data buffers
N = 200  # Number of points to show
fs = 20  # Sample rate in Hz
time_axis = [-(N - 1 - i) / fs for i in range(N)]  # left: -10, right: 0 for N=200, fs=20

thermistor_vals = deque([0]*N, maxlen=N)
strain_vals = deque([0]*N, maxlen=N)
fused_vals = deque([0]*N, maxlen=N)
thermistor_rate_vals = deque([0]*N, maxlen=N)
strain_rate_vals = deque([0]*N, maxlen=N)
breathT_flags = deque([0]*N, maxlen=N)
breathS_flags = deque([0]*N, maxlen=N)

# Initialize plot with gridline and labels
plt.ion()
fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)

line1, = axs[0].plot(time_axis, thermistor_vals, label='Thermistor Voltage')
breathT_scatter = axs[0].scatter([], [], color='red', marker='o', label='Breath Detected')
axs[0].set_ylabel('Voltage')
axs[0].set_title('Thermistor')
axs[0].set_ylim(-1, 1)
axs[0].legend(loc='upper right')

line2, = axs[1].plot(time_axis, strain_vals, label='Strain Voltage', color='orange')
breathS_scatter = axs[1].scatter([], [], color='red', marker='o', label='Breath Detected')
axs[1].set_ylabel('Voltage')
axs[1].set_title('Strain Gauge')
axs[1].set_ylim(-0.8, 0.8)
axs[1].legend(loc='upper right')

line3, = axs[2].plot(time_axis, fused_vals, label='Fused Breath Rate', color='green')
axs[2].set_ylabel('Breaths/min')
axs[2].set_title('Fused Breath Rate (BPM)')
axs[2].set_ylim(0, 40)
axs[2].legend(loc='upper right')
axs[2].set_xlabel('Time (s)')

# Add a text handle for displaying the current breath rate numerically in top left corner
rate_text = axs[2].text(
    0.02, 0.96, '', transform=axs[2].transAxes,
    fontsize=12, color='blue', ha='left', va='top',
    bbox=dict(facecolor='white', alpha=0.7, edgecolor='none')
)

# Set x-axis limits for all subplots
for ax in axs:
    ax.set_xlim(time_axis[0], time_axis[-1])
    ax.grid(True)

plt.tight_layout()

PLOT_EVERY_N = 10  # Only update plot every N samples
counter = 0

samples_per_record = 20 * fs  # 20 seconds * sample rate
sample_counter = 0
estimates = []
fileName = "file5.csv"  # File to save estimates

while True:
    try:
        line = ser.readline().decode().strip()
        if not line:
            continue
        parts = line.split(',')
        if len(parts) != 7:
            continue
        therm, strain, fused, therm_rate, strain_rate, breathT, breathS = parts
        therm = float(therm)
        strain = float(strain)
        fused = float(fused)
        therm_rate = float(therm_rate)
        strain_rate = float(strain_rate)
        breathT = int(breathT)
        breathS = int(breathS)
        thermistor_vals.append(therm)
        strain_vals.append(strain)
        fused_vals.append(fused)
        thermistor_rate_vals.append(therm_rate)
        strain_rate_vals.append(strain_rate)
        breathT_flags.append(breathT)
        breathS_flags.append(breathS)

        sample_counter += 1

        # Record every 30 seconds based on sample count
        if sample_counter >= samples_per_record:
            estimates.append([len(estimates) * 20, fused, therm_rate, strain_rate])  # [elapsed seconds, fused, thermistor, strain]
            sample_counter = 0
            # Write to CSV each time to avoid data loss
            with open(fileName, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["ElapsedSeconds", "FusedEstimate", "ThermistorRate", "StrainRate"])
                writer.writerows(estimates)
            print(f"Estimates recorded: {len(estimates)}")

        counter += 1
        if counter % PLOT_EVERY_N == 0:
            line1.set_ydata(thermistor_vals)
            line2.set_ydata(strain_vals)
            line3.set_ydata(fused_vals)

            # Update breath detected scatter for thermistor
            xT = [time_axis[i] for i, flag in enumerate(breathT_flags) if flag]
            yT = [thermistor_vals[i] for i in range(len(breathT_flags)) if breathT_flags[i]]
            breathT_scatter.remove()
            breathT_scatter = axs[0].scatter(xT, yT, color='red', marker='o', label='Breath Detected')

            # Update breath detected scatter for strain
            xS = [time_axis[i] for i, flag in enumerate(breathS_flags) if flag]
            yS = [strain_vals[i] for i in range(len(breathS_flags)) if breathS_flags[i]]
            breathS_scatter.remove()
            breathS_scatter = axs[1].scatter(xS, yS, color='red', marker='o', label='Breath Detected')

            # Update the current breath rate estimate text
            rate_text.set_text(f"Current Estimate: {fused:.2f} BPM")

            plt.pause(0.001)
    except KeyboardInterrupt:
         # Save estimates on exit
        with open(fileName, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["ElapsedSeconds", "FusedEstimate", "ThermistorRate", "StrainRate"])
            writer.writerows(estimates)
        break
    except Exception as e:
        print("Error:", e)