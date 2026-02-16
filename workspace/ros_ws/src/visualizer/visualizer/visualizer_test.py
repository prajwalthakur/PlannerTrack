import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pdb
def plot_ellipsoid(ax, center, radii, color='blue', alpha=0.5):
    u = np.linspace(0, 2 * np.pi/2, 30)
    v = np.linspace(0, np.pi/2, 30)
    x = center[0] + radii[0] * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radii[1] * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radii[2] * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x, y, z, color=color, alpha=alpha, linewidth=0)

def plot_cylinder(ax, center, radius_xy, height, color='blue', alpha=0.5):
    z = np.linspace(center[2], center[2] + height, 30)
    theta = np.linspace(0, 2 * np.pi, 30)
    theta, z = np.meshgrid(theta, z)
    pdb.set_trace()
    x = center[0] + radius_xy[0] * np.cos(theta)
    y = center[1] + radius_xy[1] * np.sin(theta)
    ax.plot_surface(x, y, z, color=color, alpha=alpha, linewidth=0)

# Initialize plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(0, 2.5)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.view_init(elev=30, azim=45)
ax.set_title("Obstacle Shape Visualization")

# Parameters
z_lim = 2.2

# Example 1: Ellipsoid
center_ellipsoid = (0, -1, 0.5)
radii_ellipsoid = (0.5, 0.3, 0.4)
plot_ellipsoid(ax, center_ellipsoid, radii_ellipsoid, color='cyan')

# Example 2: Tall obstacle (cylinder)
center_cylinder = (0, 1, 0.3)
radius_xy = (0.4, 0.4)
height = 2.0
plot_cylinder(ax, center_cylinder, radius_xy, height, color='steelblue')

# Show red boundary for z_lim
ax.plot([2, 2], [2, 2], [0, z_lim], color='red', linestyle='--', linewidth=2)
ax.plot([2], [2], [z_lim], marker='o', color='red')

plt.tight_layout()
plt.show()
