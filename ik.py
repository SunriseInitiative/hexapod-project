import math

def inverse_kinematics_2link_3d(x, y, z, L1, L2):
    # Step 1: Base angle (around Y-axis)
    theta_base = math.atan2(x, z)  # rotation around vertical axis

    # Step 2: Distance from base to target
    dxz = math.sqrt(x**2 + z**2)
    d = math.sqrt(dxz**2 + y**2)  # full 3D distance

    # Clamp if out of reach
    max_reach = L1 + L2
    if d > max_reach:
        d = max_reach

    # Step 3: Elbow angle using Law of Cosines
    cos_elbow = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))  # clamp due to float error
    theta_elbow = math.acos(cos_elbow)

    # Step 4: Shoulder angle
    cos_shoulder = (d**2 + L1**2 - L2**2) / (2 * d * L1)
    cos_shoulder = max(-1.0, min(1.0, cos_shoulder))  # clamp
    theta_shoulder_offset = math.acos(cos_shoulder)
    theta_shoulder_elevation = math.atan2(y, dxz)

    theta_shoulder = theta_shoulder_elevation + theta_shoulder_offset

    # Convert to degrees (optional)
    return {
        "theta_base": math.degrees(theta_base),
        "theta_shoulder": math.degrees(theta_shoulder),
        "theta_elbow": math.degrees(theta_elbow)
    }



import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

def plot_arm_3d(x, y, z, L1, L2, angles):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    def update(frame):
        theta_base_rad = math.radians(angles["theta_base"])
        theta_shoulder_rad = math.radians(angles["theta_shoulder"])
        theta_elbow_rad = math.radians(angles["theta_elbow"])

        base_x = 0
        base_y = 0
        base_z = 0

        shoulder_x = L1 * math.cos(theta_shoulder_rad) * math.sin(theta_base_rad)
        shoulder_y = L1 * math.sin(theta_shoulder_rad)
        shoulder_z = L1 * math.cos(theta_shoulder_rad) * math.cos(theta_base_rad)

        elbow_x = shoulder_x + L2 * math.cos(theta_elbow_rad) * math.sin(theta_base_rad)
        elbow_y = shoulder_y + L2 * math.sin(theta_elbow_rad)
        elbow_z = shoulder_z + L2 * math.cos(theta_elbow_rad) * math.cos(theta_base_rad)

        ax.clear()
        ax.plot([base_x, shoulder_x, elbow_x], [base_y, shoulder_y, elbow_y], [base_z, shoulder_z, elbow_z], 'o-', markersize=8, linewidth=3)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim([-L1-L2, L1+L2])
        ax.set_ylim([-L1-L2, L1+L2])
        ax.set_zlim([-L1-L2, L1+L2])
        ax.set_title('2-Link 3D Arm')

    ani = animation.FuncAnimation(fig, update, interval=100, repeat=True)
    plt.show()

for i in range(10):
    for j in range(10):
        for k in range(10):
            x,y,z = i+1, j+1, k+1
            angles = inverse_kinematics_2link_3d(x=x/10, y=y/10, z=z/10, L1=0.6, L2=0.5)
            plot_arm_3d(x=x/10, y=y/10, z=z/10, L1=0.6, L2=0.5, angles=angles)
            plt.pause(1)  # Pause for a short time to see the plot
            plt.clf()  # Clear the current figure

print("donezies with the funzies")
