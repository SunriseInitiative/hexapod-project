import ik
import time
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

last_line = 0

with open("programMemory.txt", "r+") as f:
        lines = f.readlines()
        if len(lines) > 0:
            last_line = lines[len(lines) - 1].strip()
            f.write(f"{int(last_line) + 1}\n\n")
with open("log.txt", "a") as f:
    f.write(f"===={last_line}====\n")

def tween3D(start, end, intervals, precision=6):
    endList = []
    intervalDistance = ((end[0] - start[0]) / intervals, (end[1] - start[1]) / intervals, (end[2] - start[2]) / intervals)
    if start == end:
        return start
    if len(start) == 3 and len(end) == 3:
        for i in range(intervals + 1):  # Include the endpoint
            x = round(start[0] + (intervalDistance[0] * i), precision)
            y = round(start[1] + (intervalDistance[1] * i), precision)
            z = round(start[2] + (intervalDistance[2] * i), precision)
            endList.append((x, y, z))
        return endList
    else:
        return False

def animate_walk(tween_points, L1, L2, interval_ms):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Initialize the plot with empty data
    line, = ax.plot([], [], [], 'o-', markersize=8, linewidth=3)

    def update(frame):
        point = tween_points[frame]
        angles = ik.inverseKinematics2Link3D(point[0], point[1], point[2], L1, L2)

        theta_base_rad = math.radians(angles["theta_base"])
        theta_shoulder_rad = math.radians(angles["theta_shoulder"])
        theta_elbow_rad = math.radians(angles["theta_elbow"])

        base_x, base_y, base_z = 0, 0, 0
        shoulder_x = L1 * math.cos(theta_shoulder_rad) * math.sin(theta_base_rad)
        shoulder_y = L1 * math.sin(theta_shoulder_rad)
        shoulder_z = L1 * math.cos(theta_shoulder_rad) * math.cos(theta_base_rad)
        elbow_x = shoulder_x + L2 * math.cos(theta_elbow_rad) * math.sin(theta_base_rad)
        elbow_y = shoulder_y + L2 * math.sin(theta_elbow_rad)
        elbow_z = shoulder_z + L2 * math.cos(theta_elbow_rad) * math.cos(theta_base_rad)

        # Update the line data
        line.set_data_3d([base_x, shoulder_x, elbow_x], [base_y, shoulder_y, elbow_y], [base_z, shoulder_z, elbow_z])
        return line,

    # Set up the axes limits and labels
    ax.set_xlim([-L1 - L2, L1 + L2])
    ax.set_ylim([-L1 - L2, L1 + L2])
    ax.set_zlim([-L1 - L2, L1 + L2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Hexapod Walk Animation')

    ani = animation.FuncAnimation(fig, update, frames=len(tween_points), interval=interval_ms, blit=True, repeat=False)
    plt.show()

# Generate tween points
tween_points = tween3D((0.1, 0.1, 0.1), (1, 1, 1), 10)

# Animate the walk
animate_walk(tween_points, L1=0.6, L2=0.5, interval_ms=1000)