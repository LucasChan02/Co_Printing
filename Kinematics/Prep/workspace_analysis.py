import numpy as np
import itertools
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from robot_data_provided import robot
from forward_kinematics import forward_kinematics

def compute_workspace( num_samples ):
    # Extract joint ranges from the robot's configuration
    joint_ranges = []
    for body_id in range(1, 5): 
        joint_min = robot.body[body_id].joint_range[0]
        joint_max = robot.body[body_id].joint_range[1]
        print(joint_min, joint_max)
        joint_ranges.append((joint_min, joint_max))
    
    # Generate grid of joint angles (using itertools.product)
    angle_grids = [np.linspace(min_max[0], min_max[1], num_samples) for min_max in joint_ranges]
    print(np.shape(angle_grids))
    all_angle_combinations = itertools.product(*angle_grids)
    
    # Compute end-effector positions for all combinations
    positions = []
    angle_diffs = []
    for i, q in enumerate(all_angle_combinations):
        q_array = np.array(q)
        sol, _ = forward_kinematics(q_array)  # Compute FK
        positions.append(sol.end_eff_pos)
        theta = np.arccos(sol.end_eff_rot[2,2]) 
        angle_diffs.append(theta)
        
        # Print progress
        if (i + 1) % 10000 == 0:
            print(f"Processed {i + 1} configurations...")
    
    return np.array(positions), np.array(angle_diffs)

def plot_workspace(positions, angle_diffs):
    # Plot the 3D workspace
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(
        positions[:, 0], positions[:, 1], positions[:, 2],
        s=0.2, alpha=0.4, c=angle_diffs, cmap='RdYlGn_r'
    )
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Robot Reachable Workspace')
    
    # Set equal aspect ratio
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
    
    # Enable interactive controls
    ax.mouse_init()
    plt.show()



# def plot_workspace(positions):
#     # Plot the 3D workspace
#     if positions.ndim == 1 and positions.shape[0] == 0 : # check if positions is empty
#         print("No positions to plot.")
#         return
#     if positions.shape[0] == 0: # check if positions is empty array of shape (0,3) for instance
#         print("No positions to plot.")
#         return

#     fig = plt.figure(figsize=(10, 8))
#     ax = fig.add_subplot(111, projection='3d')
#     ax.scatter(
#         positions[:, 0], positions[:, 1], positions[:, 2],
#         s=1, alpha=0.2, c=positions[:, 2], cmap='viridis' # Increased s and alpha for better visibility
#     )
#     ax.set_xlabel('X (m)')
#     ax.set_ylabel('Y (m)')
#     ax.set_zlabel('Z (m)')
#     ax.set_title('Robot Reachable Workspace')
    
#     # Set equal aspect ratio for a more representative plot
#     x_limits = ax.get_xlim3d()
#     y_limits = ax.get_ylim3d()
#     z_limits = ax.get_zlim3d()

#     x_range = abs(x_limits[1] - x_limits[0])
#     x_middle = np.mean(x_limits)
#     y_range = abs(y_limits[1] - y_limits[0])
#     y_middle = np.mean(y_limits)
#     z_range = abs(z_limits[1] - z_limits[0])
#     z_middle = np.mean(z_limits)

#     plot_radius = 0.5 * max([x_range, y_range, z_range])

#     ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
#     ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
#     ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
    
#     plt.show()

if __name__ == '__main__':
    # Adjust num_samples for resolution (higher = slower)
    positions, angle_diffs = compute_workspace(num_samples = 12)  # samples per joint
    plot_workspace(positions, angle_diffs)