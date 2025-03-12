import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os

# Define Data Path (Change this to match your CSV file location)
DATA_PATH = "/home/sahan/Projects/Ros2Visual/robot_trajectory_data_20250312_021545.csv"
OUTPUT_DIR = "./plots"

# Ensure the output directory exists
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Load the data
df = pd.read_csv(DATA_PATH)

# Convert timestamp to relative time
df['time'] = df['timestamp'] - df['timestamp'].min()

# Set Seaborn style
sns.set_style("whitegrid")

### 1. Robot Path Visualization (Now with Waypoints!)
plt.figure(figsize=(10, 8))
sns.scatterplot(data=df, x='robot_x', y='robot_y', alpha=0.6, hue='waypoint_number', palette='viridis')
sns.scatterplot(data=df, x='goal_x', y='goal_y', marker="X", color='red', s=100)  # Highlight waypoints

# Connect waypoints to show path structure
for _, group in df.groupby('waypoint_number'):
    plt.plot(group['robot_x'], group['robot_y'], linestyle="--", alpha=0.7)

plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Robot Path with Waypoints')
plt.legend()
plt.axis('equal')
plt.tight_layout()
plt.savefig(f"{OUTPUT_DIR}/robot_path.png", dpi=300, bbox_inches='tight')
plt.close()

### 2. Distance Error Over Time
plt.figure(figsize=(10, 6))
sns.lineplot(data=df, x='time', y='distance_error', color='blue', label='Distance Error')
plt.xlabel('Time (s)')
plt.ylabel('Distance Error (m)')
plt.title('Distance Error Over Time')
plt.legend()
plt.tight_layout()
plt.savefig(f"{OUTPUT_DIR}/distance_error.png", dpi=300, bbox_inches='tight')
plt.close()

### 3. Heading Error Over Time
plt.figure(figsize=(10, 6))
sns.lineplot(data=df, x='time', y='heading_error', color='red', label='Heading Error')
plt.xlabel('Time (s)')
plt.ylabel('Heading Error (radians)')
plt.title('Heading Error Over Time')
plt.legend()
plt.tight_layout()
plt.savefig(f"{OUTPUT_DIR}/heading_error.png", dpi=300, bbox_inches='tight')
plt.close()

### 4. Linear Velocity Over Time
plt.figure(figsize=(10, 6))
sns.lineplot(data=df, x='time', y='linear_velocity', color='green', label='Linear Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Linear Velocity (m/s)')
plt.title('Linear Velocity Over Time')
plt.legend()
plt.tight_layout()
plt.savefig(f"{OUTPUT_DIR}/linear_velocity.png", dpi=300, bbox_inches='tight')
plt.close()

### 5. Angular Velocity Over Time
plt.figure(figsize=(10, 6))
sns.lineplot(data=df, x='time', y='angular_velocity', color='purple', label='Angular Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Angular Velocity Over Time')
plt.legend()
plt.tight_layout()
plt.savefig(f"{OUTPUT_DIR}/angular_velocity.png", dpi=300, bbox_inches='tight')
plt.close()

### 6. PID Components for Distance Control
plt.figure(figsize=(10, 6))
sns.lineplot(data=df, x='time', y='P_linear', label='P Component', color='blue')
sns.lineplot(data=df, x='time', y='I_linear', label='I Component', color='green')
sns.lineplot(data=df, x='time', y='D_linear', label='D Component', color='red')
plt.xlabel('Time (s)')
plt.ylabel('PID Values')
plt.title('PID Components for Linear Control')
plt.legend()
plt.tight_layout()
plt.savefig(f"{OUTPUT_DIR}/pid_linear.png", dpi=300, bbox_inches='tight')
plt.close()

### 7. PID Components for Heading Control
plt.figure(figsize=(10, 6))
sns.lineplot(data=df, x='time', y='P_heading', label='P Component', color='blue')
sns.lineplot(data=df, x='time', y='I_heading', label='I Component', color='green')
sns.lineplot(data=df, x='time', y='D_heading', label='D Component', color='red')
plt.xlabel('Time (s)')
plt.ylabel('PID Values')
plt.title('PID Components for Heading Control')
plt.legend()
plt.tight_layout()
plt.savefig(f"{OUTPUT_DIR}/pid_heading.png", dpi=300, bbox_inches='tight')
plt.close()

print(f"✅ Plots saved in {OUTPUT_DIR}")

