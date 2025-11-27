import os
import glob
import pandas as pd
import matplotlib.pyplot as plt

# Path to your logs
log_dir = "/home/nila/ros2_ws/install/racing_perception/lib/python3.10/site-packages/racing_perception/logs"

files = sorted(glob.glob(os.path.join(log_dir, "run_*.csv")))
if not files:
    print("No log files found in", log_dir)
    raise SystemExit

latest = files[-1]
print("Loading:", latest)

df = pd.read_csv(latest)

# Forward fill missing values
df.ffill(inplace=True)

# Ensure numeric columns are numeric
numeric_cols = [
    "time",
    "corridor_width",
    "safe_speed",
    "steering_cmd",
    "cmd_vel_linear",
    "cmd_vel_angular",
]
for col in numeric_cols:
    df[col] = pd.to_numeric(df[col], errors="coerce")

# Convert to numpy arrays to avoid pandas indexing issues
t = df["time"].to_numpy()

corr = df["corridor_width"].to_numpy()
speed = df["safe_speed"].to_numpy()
steer = df["steering_cmd"].to_numpy()
lin = df["cmd_vel_linear"].to_numpy()
ang = df["cmd_vel_angular"].to_numpy()

# Make sure output folder is this analysis folder
out_dir = "."
print("Saving plots to", os.path.abspath(out_dir))

# Plot 1: Corridor width
plt.figure(figsize=(10, 4))
plt.plot(t, corr)
plt.title("Corridor Width Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Width (m)")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(out_dir, "corridor_width.png"), dpi=150)
plt.close()

# Plot 2: Recommended Speed
plt.figure(figsize=(10, 4))
plt.plot(t, speed)
plt.title("Recommended Speed Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(out_dir, "safe_speed.png"), dpi=150)
plt.close()

# Plot 3: Steering Command
plt.figure(figsize=(10, 4))
plt.plot(t, steer)
plt.title("Steering Command Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Steer (-1=left, +1=right)")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(out_dir, "steering_cmd.png"), dpi=150)
plt.close()

# Plot 4: Linear Speed
plt.figure(figsize=(10, 4))
plt.plot(t, lin)
plt.title("Actual Linear Speed Over Time")
plt.xlabel("Time (s)")
plt.ylabel("linear.x (m/s)")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(out_dir, "linear_speed.png"), dpi=150)
plt.close()

# Plot 5: Angular Speed
plt.figure(figsize=(10, 4))
plt.plot(t, ang)
plt.title("Angular Velocity Over Time")
plt.xlabel("Time (s)")
plt.ylabel("angular.z (rad/s)")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(out_dir, "angular_speed.png"), dpi=150)
plt.close()

print("Plots saved.")
