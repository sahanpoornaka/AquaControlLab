import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Replace with your CSV file path
DATA_PATH = "~/Projects/Ros2Visual/trajectories/robot_trajectory_20250220_135133.csv"

# Read the CSV file
df = pd.read_csv(DATA_PATH)

# Create the plot
plt.figure(figsize=(10, 10))

# Create scatter plot for actual path
sns.scatterplot(data=df, 
                x='robot_x', 
                y='robot_y',
                alpha=0.6,
                color='steelblue',
                label='Actual Path')

# Plot ideal path (straight line from start to end)
plt.plot([df['ideal_x'].iloc[0], df['ideal_x'].iloc[-1]], 
         [df['ideal_y'].iloc[0], df['ideal_y'].iloc[-1]], 
         color='red', 
         linestyle='--', 
         alpha=0.8,
         label='Ideal Path')

# Customize the plot
plt.xlabel('X Position (m)', fontsize=12)
plt.ylabel('Y Position (m)', fontsize=12)
plt.title('Robot Path Tracking', fontsize=14)

# Add error statistics to the plot
mean_error = df['error'].mean()
max_error = df['error'].max()
stats_text = f'Mean Error: {mean_error:.3f}m\nMax Error: {max_error:.3f}m'
plt.text(0.02, 0.98, stats_text,
         transform=plt.gca().transAxes,
         verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

# Add legend
plt.legend(loc='upper right')

# Make the plot square
plt.axis('equal')

# Add grid
plt.grid(True, alpha=0.3)

# Add margins
plt.margins(0.1)

# Adjust layout
plt.tight_layout()

# Save the plot
plt.savefig('robot_path_plot.png', dpi=300, bbox_inches='tight')

# Show additional analysis
print("\nTrajectory Analysis:")
print(f"Total distance traveled: {df['robot_x'].iloc[-1]:.2f}m")
print(f"Average error: {mean_error:.3f}m")
print(f"Maximum error: {max_error:.3f}m")
print(f"Standard deviation of error: {df['error'].std():.3f}m")

# Optional: Plot error over time
plt.figure(figsize=(10, 5))
plt.plot(df.index, df['error'], color='steelblue')
plt.xlabel('Time Steps')
plt.ylabel('Error (m)')
plt.title('Path Error Over Time')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('error_plot.png', dpi=300)

# Show the plots
plt.show()
