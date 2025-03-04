import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

DATA_PATH = "~/Projects/Ros2Visual/trajectories/robot_trajectory_20250220_132847.csv"


# Read the CSV file
df = pd.read_csv(DATA_PATH)

# Filter out waypoint_number = 1
df = df[df['waypoint_number'] != 1]

# Create the plot with a light color scheme
plt.figure(figsize=(10, 10))

# Set the style
# plt.style.use('seaborn-whitegrid')

# Create scatter plot
sns.scatterplot(data=df, 
                x='robot_x', 
                y='robot_y',
                alpha=0.6,
                color='steelblue')

# Add ideal lines
for _, group in df.groupby('waypoint_number'):
    start_x = group['ideal_line_start_x'].iloc[0]
    start_y = group['ideal_line_start_y'].iloc[0]
    end_x = group['ideal_line_end_x'].iloc[0]
    end_y = group['ideal_line_end_y'].iloc[0]
    plt.plot([start_x, end_x], [start_y, end_y], 
             color='red', 
             linestyle='--', 
             alpha=0.8,
             label=f'Ideal Line {int(_)}')

# Customize the plot
plt.xlabel('X Position (m)', fontsize=12)
plt.ylabel('Y Position (m)', fontsize=12)
plt.title('Robot Path with Ideal Lines', fontsize=14)

# Add legend
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')

# Make the plot square
plt.axis('equal')

# Add margins
plt.margins(0.1)

# Adjust layout to prevent label cutoff
plt.tight_layout()

# Save the plot
plt.savefig('robot_path_plot.png', dpi=300, bbox_inches='tight')

# Show the plot
plt.show()
