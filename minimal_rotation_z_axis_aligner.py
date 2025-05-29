import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D

def rotation_matrix_intrinsic_ZYZ(alpha, beta, gamma):
    """Calculate the rotation matrix for intrinsic Z-Y-Z Euler angles"""
    ca, sa = np.cos(alpha), np.sin(alpha)
    cb, sb = np.cos(beta), np.sin(beta)
    cg, sg = np.cos(gamma), np.sin(gamma)
    
    R = np.array([
        [cg*ca - sg*sa*cb, -cg*sa - sg*ca*cb,  sg*sb],
        [sg*ca + cg*sa*cb, -sg*sa + cg*ca*cb, -cg*sb],
        [sa*sb,            ca*sb,             cb]
    ])
    return R

def align_z_axis_to_global_z(R):
    """Align the Z-axis of the rotation matrix to the global Z-axis [0,0,1]"""
    z_current = R[:, 2]
    z_target = np.array([0, 0, 1])
    
    # Calculate rotation axis
    k = np.cross(z_current, z_target)
    
    # Handle special cases
    if np.linalg.norm(k) < 1e-10:
        if np.dot(z_current, z_target) > 0:
            return R
        else:
            if abs(z_current[0]) < abs(z_current[1]):
                v = np.array([1, 0, 0])
            else:
                v = np.array([0, 1, 0])
            k = np.cross(z_current, v)
            k = k / np.linalg.norm(k)
            
            theta = np.pi
            K = np.array([
                [0, -k[2], k[1]],
                [k[2], 0, -k[0]],
                [-k[1], k[0], 0]
            ])
            R_align = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
            return R_align @ R
    
    # Normalize rotation axis
    k = k / np.linalg.norm(k)
    
    # Calculate rotation angle
    cos_theta = np.dot(z_current, z_target)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    
    # Build skew-symmetric matrix
    K = np.array([
        [0, -k[2], k[1]],
        [k[2], 0, -k[0]],
        [-k[1], k[0], 0]
    ])
    
    # Rodrigues' formula
    R_align = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
    
    return R_align @ R

def plot_coordinate_system(ax, R, origin, color, label, scale=1.0, linewidth=1.5):
    """Plot a coordinate system"""
    # Axis colors: X-red, Y-green, Z-blue
    colors = [color + 'r', color + 'g', color + 'b']
    labels = [f'{label}X', f'{label}Y', f'{label}Z']
    
    for i in range(3):
        end_point = origin + R[:, i] * scale
        ax.plot([origin[0], end_point[0]],
                [origin[1], end_point[1]],
                [origin[2], end_point[2]],
                colors[i], linewidth=linewidth, label=labels[i])

def update(val):
    """Update visualization"""
    # Get slider values (degrees to radians)
    alpha = np.deg2rad(slider_alpha.val)
    beta = np.deg2rad(slider_beta.val)
    gamma = np.deg2rad(slider_gamma.val)
    
    # Calculate rotation matrix
    R = rotation_matrix_intrinsic_ZYZ(alpha, beta, gamma)
    
    # Align Z-axis
    R_aligned = align_z_axis_to_global_z(R)
    
    # Clear current plot
    ax.clear()
    
    # Set plot properties
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_xlabel('X')
    ax.set_zlim([0, 3])
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Rotation Matrix Z-Axis Alignment Visualization')
    
    # Plot global coordinate system
    plot_coordinate_system(ax, np.eye(3), np.array([0, 0, 0]), '', 'Global', 1.2, 1.0)
    
    # Plot original coordinate system (thin lines)
    plot_coordinate_system(ax, R, np.array([0, 0, 0]), ':', 'Original', 1.0, 1.0)
    
    # Plot aligned coordinate system (thick lines)
    plot_coordinate_system(ax, R_aligned, np.array([0, 0, 0]), '-', 'Aligned', 1.0, 2.5)
    
    # Calculate Z-axis alignment error
    z_original = R[:, 2]
    z_aligned = R_aligned[:, 2]
    alignment_error = np.degrees(np.arccos(np.clip(np.dot(z_aligned, np.array([0, 0, 1])), -1.0, 1.0)))
    
    # Display information
    ax.text(0, 0, 2.8, f'Original Z-axis: [{z_original[0]:.3f}, {z_original[1]:.3f}, {z_original[2]:.3f}]')
    ax.text(0, 0, 2.6, f'Aligned Z-axis: [{z_aligned[0]:.3f}, {z_aligned[1]:.3f}, {z_aligned[2]:.3f}]')
    ax.text(0, 0, 2.4, f'Z-axis Alignment Error: {alignment_error:.3f}°')
    
    # Show legend
    ax.legend(loc='upper right')
    
    # Update plot
    fig.canvas.draw_idle()

if __name__ == '__main__':
        
    # Create figure
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    plt.subplots_adjust(bottom=0.25)

    # Create sliders
    ax_alpha = plt.axes([0.25, 0.15, 0.65, 0.03])
    ax_beta = plt.axes([0.25, 0.10, 0.65, 0.03])
    ax_gamma = plt.axes([0.25, 0.05, 0.65, 0.03])

    slider_alpha = Slider(ax_alpha, 'α (Rotation around Z)', 0, 360, valinit=45)
    slider_beta = Slider(ax_beta, 'β (Rotation around new Y)', 0, 180, valinit=45)
    slider_gamma = Slider(ax_gamma, 'γ (Rotation around newest Z)', 0, 360, valinit=45)

    # Register update function
    slider_alpha.on_changed(update)
    slider_beta.on_changed(update)
    slider_gamma.on_changed(update)

    # Create reset button
    reset_ax = plt.axes([0.05, 0.05, 0.1, 0.04])
    button = Button(reset_ax, 'Reset', hovercolor='0.975')

    def reset(event):
        slider_alpha.reset()
        slider_beta.reset()
        slider_gamma.reset()

    button.on_clicked(reset)

    # Initialize display
    update(None)

    # Show plot
    plt.show()