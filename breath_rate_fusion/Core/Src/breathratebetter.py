import serial
import matplotlib.pyplot as plt
from collections import deque

# Serial port settings
ser = serial.Serial('COM6', 115200, timeout=1)  # Replace 'COMx' with your port

# Data buffers
N = 200  # Number of points to show
thermistor_vals = deque([0]*N, maxlen=N)
strain_vals = deque([0]*N, maxlen=N)
fused_vals = deque([0]*N, maxlen=N)
breathT_flags = deque([0]*N, maxlen=N)
breathS_flags = deque([0]*N, maxlen=N)

# Initialize plot with gridline and labels
plt.ion()
fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)

line1, = axs[0].plot(thermistor_vals, label='Thermistor Voltage')
breathT_scatter = axs[0].scatter([], [], color='red', marker='o', label='Breath Detected')
axs[0].set_ylabel('Voltage (V)')
axs[0].set_title('Thermistor (Filtered)')
axs[0].set_ylim(-0.8, 0.8)
axs[0].legend(loc='upper right')

line2, = axs[1].plot(strain_vals, label='Strain Voltage', color='orange')
breathS_scatter = axs[1].scatter([], [], color='red', marker='o', label='Breath Detected')
axs[1].set_ylabel('Voltage (V)')
axs[1].set_title('Strain Gauge (Filtered)')
axs[1].set_ylim(-0.8, 0.8)
axs[1].legend(loc='upper right')

line3, = axs[2].plot(fused_vals, label='Fused Breath Rate', color='green')
axs[2].set_ylabel('Breaths/min')
axs[2].set_title('Fused Breath Rate (BPM)')
axs[2].set_ylim(0, 40)
axs[2].legend(loc='upper right')
axs[2].set_xlabel('Sample')

# Add gridlines to each subplot
for ax in axs:
    ax.grid(True)

plt.tight_layout()

PLOT_EVERY_N = 10  # Only update plot every N samples
counter = 0

while True:
    try:
        line = ser.readline().decode().strip()
        if not line:
            continue
        parts = line.split(',')
        if len(parts) != 5:
            continue
        therm, strain, fused, breathT, breathS = parts
        therm = float(therm)
        strain = float(strain)
        fused = float(fused)
        breathT = int(breathT)
        breathS = int(breathS)
        thermistor_vals.append(therm)
        strain_vals.append(strain)
        fused_vals.append(fused)
        breathT_flags.append(breathT)
        breathS_flags.append(breathS)

        counter += 1
        if counter % PLOT_EVERY_N == 0:
            line1.set_ydata(thermistor_vals)
            line2.set_ydata(strain_vals)
            line3.set_ydata(fused_vals)

            # Update breath detected scatter for thermistor
            xT = [i for i, flag in enumerate(breathT_flags) if flag]
            yT = [thermistor_vals[i] for i in xT]
            breathT_scatter.remove()
            breathT_scatter = axs[0].scatter(xT, yT, color='red', marker='o', label='Breath Detected')

            # Update breath detected scatter for strain
            xS = [i for i, flag in enumerate(breathS_flags) if flag]
            yS = [strain_vals[i] for i in xS]
            breathS_scatter.remove()
            breathS_scatter = axs[1].scatter(xS, yS, color='red', marker='o', label='Breath Detected')

            for ax in axs:
                ax.relim()
                ax.autoscale_view()
            plt.pause(0.001)
    except KeyboardInterrupt:
        break
    except Exception as e:
        print("Error:", e)