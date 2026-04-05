import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# ===== LOAD DATA =====
df = pd.read_csv("swarm_data.csv")

# ===== REMOVE STARTUP (first few seconds) =====
df = df[df["time"] > 3.0]

# ===== RAW DATA (for plots) =====
time_all = df["time"]

r1_x_all, r1_y_all = df["r1_x"], df["r1_y"]
r2_x_all, r2_y_all = df["r2_x"], df["r2_y"]
r3_x_all, r3_y_all = df["r3_x"], df["r3_y"]

# ===== DISTANCE (FULL DATA for plotting) =====
d12_all = np.sqrt((r1_x_all - r2_x_all)**2 + (r1_y_all - r2_y_all)**2)
d13_all = np.sqrt((r1_x_all - r3_x_all)**2 + (r1_y_all - r3_y_all)**2)

# ===== ESTIMATE FORMATION OFFSET (steady region) =====
steady_df = df[df["time"] > df["time"].max() * 0.6]

r1_x_s, r1_y_s = steady_df["r1_x"], steady_df["r1_y"]
r2_x_s, r2_y_s = steady_df["r2_x"], steady_df["r2_y"]
r3_x_s, r3_y_s = steady_df["r3_x"], steady_df["r3_y"]

d12_s = np.sqrt((r1_x_s - r2_x_s)**2 + (r1_y_s - r2_y_s)**2)
d13_s = np.sqrt((r1_x_s - r3_x_s)**2 + (r1_y_s - r3_y_s)**2)

desired_offset = np.mean(d12_s)

# ===== DISTANCE ERROR (FULL DATA) =====
error_all = (np.abs(d12_all - desired_offset) + np.abs(d13_all - desired_offset)) / 2

# ===== METRICS (STEADY STATE ONLY) =====
error_steady = (np.abs(d12_s - desired_offset) + np.abs(d13_s - desired_offset)) / 2
mean_error = np.mean(error_steady)

oscillation_amp = np.max(r2_y_s) - np.min(r2_y_s)

# ===== CONVERGENCE TIME =====
threshold = 0.05
convergence_time = None

for i in range(len(error_all)):
    if error_all.iloc[i] < threshold:
        convergence_time = time_all.iloc[i]
        break

# ===== PRINT RESULTS =====
print("===== RESULTS =====")
print(f"Estimated Formation Offset: {desired_offset:.3f} m")
print(f"Mean Distance Error (steady): {mean_error:.4f} m")
print(f"Oscillation Amplitude (steady): {oscillation_amp:.4f} m")

if convergence_time is not None:
    print(f"Convergence Time: {convergence_time:.2f} s")
else:
    print("Convergence Time: Not reached")

# ===== PLOT 1: Distance Error =====
plt.figure()
plt.plot(time_all, error_all)
plt.xlabel("Time (s)")
plt.ylabel("Distance Error (m)")
plt.title("Inter-Robot Distance Error")
plt.grid()
plt.savefig("distance_error.png")

# ===== PLOT 2: Oscillation =====
plt.figure()
plt.plot(time_all, r2_y_all, label="Robot2 Y")
plt.xlabel("Time (s)")
plt.ylabel("Y Position (m)")
plt.title("Lateral Oscillation")
plt.legend()
plt.grid()
plt.savefig("oscillation.png")

# ===== PLOT 3: Trajectory =====
plt.figure()
plt.plot(r1_x_all, r1_y_all, label="Robot1")
plt.plot(r2_x_all, r2_y_all, label="Robot2")
plt.plot(r3_x_all, r3_y_all, label="Robot3")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Robot Trajectories")
plt.legend()
plt.axis("equal")
plt.grid()
plt.savefig("trajectory.png")

plt.show()