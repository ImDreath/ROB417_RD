import numpy as np
import matplotlib.pyplot as plt
import numdifftools as nd
import jacobian as jc
import draw_3d_arm as d3d
import place_3D as p3d
from scipy.integrate import solve_ivp

def draw_links(link_set, link_colors, ax):
    """
    Draws a set of lines representing a link structure into a specified axis.
    
    Parameters:
    - link_set: List of matrices, each representing the endpoints of a link in 2D or 3D space.
    - link_colors: List of colors for each link; each entry can be a color string or RGB tuple.
    - ax: Matplotlib axis to plot the links on.
    
    Returns:
    - l: List of line objects created for each link in link_set.
    """
    # Create an empty list to store line objects (handles) for each link
    l = []
    # Draw each link with circles at endpoints and specified color
    for i, link in enumerate(link_set):
        # print(link)
        # print(link_set)
        x_data = link[0, :]
        y_data = link[1, :]
        
        # If in 3D, add z_data
        if link.shape[0] == 3:
            z_data = link[2, :]
            line_handle, = ax.plot(x_data, y_data, z_data, linestyle='-', marker='o', color=link_colors[i])
        else:
            line_handle, = ax.plot(x_data, y_data, linestyle='-', marker='o', color=link_colors[i])
        
        # Append the line handle to the list
        l.append(line_handle)
    # plt.show()
    return l



def threeD_update_links(l,link_set):

    # % Update the drawings of a set of lines for a link structure
    # %
    # % Inputs:
    # %
    # %   link_set: A 1xn cell array, each entry of which is a matrix whose
    # %       columns are the endpoints of the lines describing one link of the
    # %       system (as constructed by planar_build_links or
    # %       planar_build_links_prismatic
    # %
    # % Output:
    # %
    # %   l: A cell array of the same size as link_set, in which each entry is a
    # %       handle to a surface structure for that link


    #     
    #     % Loop over the lines whose handles are in 'l', replacing their 'XData',
    #     % 'YData', and 'ZData' with the information from 'link_set'

    for i, link in enumerate(link_set):
        
        l[i].set_xdata(link[0, :])
        l[i].set_ydata(link[1, :])
        l[i].set_3d_properties(link[2, :])

    return l

def threeD_update_links2(l, link_set):
    """
    Update the drawings of a set of lines for a link structure.

    Parameters:
    l (list): A list of Line3D objects for each link.
    link_set (list): A list of numpy arrays, each containing the endpoints
                     of the lines describing one link of the system.

    Returns:
    l (list): Updated list of Line3D objects.
    """
    for i in range(len(link_set)):
        # Update the data for each line
        # l[i].set_ydata()
        # set_data(link_set[i][0, :], link_set[i][1, :])
        l[i].set_3d_properties(link_set[i][2, :])
    
    return l

def follow_trajectory(t, alpha, J, shape_to_draw):
    """
    Find the joint velocities 'alpha_dot' that make the end of an arm follow
    the shape described by 'shape_to_draw' at time 't'.

    Parameters:
    t (float): Time at which to evaluate the velocity along the shape.
    alpha (numpy array): Current joint angles of the arm.
    J (callable): Function that returns the Jacobian of the arm's end effector given 'alpha'.
    shape_to_draw (callable): Function that maps time to a 3D shape in space.

    Returns:
    alpha_dot (numpy array): Joint velocities that produce a velocity equal to d/dt(shape_to_draw) at 't'.
    v (numpy array): Derivative of 'shape_to_draw' at time 't'.
    """
    # Compute the derivative of 'shape_to_draw' at time 't' using numdifftools' Jacobian
    jacobian_shape = nd.Jacobian(shape_to_draw)
    v = jacobian_shape(t).flatten()  # Equivalent to 'v' from jacobianest

    # Compute the Jacobian at the current joint angles
    J_alpha = J(alpha)
    # J_alpha = J_alpha[0]
    # Solve for alpha_dot using least squares to approximate lsqminnorm
    alpha_dot, residuals, rank, s = np.linalg.lstsq(J_alpha, v, rcond=None)
    
    return alpha_dot, v


def circle_x(t):
    """
    Generate points on a unit circle in the y-z plane, wrapping in the
    clockwise (negative) direction around the x-axis. The range of t=[0, 1]
    corresponds to one full cycle of the circle, starting at [0, 0, 1].

    Parameters:
    t (numpy array): Array of times at which to generate points on the circle.

    Returns:
    xyz (numpy array): 3 x n matrix where each row represents the x, y, or z
                       coordinate of points on the circle, and each column
                       corresponds to one time point from t.
    """
    # Ensure t is a 1D row vector
    t = np.atleast_1d(t).flatten()
    
    # Calculate the x, y, z coordinates
    x = np.zeros_like(t)
    y = np.sin(2 * np.pi * t)
    z = np.cos(2 * np.pi * t)
    
    # Stack them into a 3 x n array
    xyz = np.vstack((x, y, z))
    
    return xyz



def ME317_Assignment_trace_circle():
    # Initialize link vectors as a list of arrays
    link_vectors = [np.array([[1], [0], [0]]), np.array([[1], [0], [0]]), np.array([[0.75], [0], [0]])]
    
    # Specify joint axes
    joint_axes = ['z', 'y', 'y']
    
    # Define the shape to draw (a scaled unit circle in the yz-plane)
    shape_to_draw = lambda t: circle_x(t) * 0.5
    
    # Define the Jacobian function
    J = lambda b: jc.arm_Jacobian(link_vectors, b, joint_axes, len(link_vectors))[0]
    
    # Define the joint velocity function
    joint_velocity = lambda t, alpha: follow_trajectory(t, alpha, J, shape_to_draw)[0]
    
    # Differential equation solver parameters
    T = (0, 1)  # time range
    a_start = np.array([0, np.pi / 4, -np.pi / 2])  # starting configuration

    # Run the ODE solver
    # sol = solve_ivp(joint_velocity, T, a_start)
    sol = solve_ivp(joint_velocity, T, a_start, t_eval=np.linspace(0, 1, 100))
    alpha = sol.y
    
    # Create figure and axes for the plot
    fig_num = 317
    ax,f = d3d.create_axes(fig_num)


    # Set colors for the links
    link_colors = ['r', 'b', 'k']
    
    # Generate the initial link set
    link_set, _, _, _, _, _, _, _ = p3d.threeD_robot_arm_links(link_vectors, alpha[:, 0], joint_axes)
    
    # Draw the links
    l = p3d.threeD_draw_links(link_set, link_colors, ax)
    
    # Prepare to draw the path traced by the robot's end-effector
    p = np.zeros((3, alpha.shape[1]))

    # Loop through each configuration
    link_end_set_with_base = []
    for i in range(alpha.shape[1]):
        # Calculate link endpoints
        _, _, _, _, _, link_end_base = d3d.threeD_robot_arm_endpoints(link_vectors, alpha[:, i], joint_axes)
        link_end_set_with_base.append(link_end_base)
        #  Save the endpoint of the last link into the corresponding column of p
        for j in range(alpha.shape[0]):
            p[j, i] = link_end_set_with_base[i][-1][j]


    
    # Plot the path of the end-effector
    l_trace, = ax.plot(p[0, :], p[1, :], p[2, :], 'k')

    # Set the view and axis properties
    ax.view_init(elev=30, azim=30)
    ax.set_box_aspect([1, 1, 1])  # Equivalent to 'vis3d' in MATLAB

    
    # Animate the arm
    link_set_history = []
    for i in range(alpha.shape[1]):
        # Update link set and plot for each timestep
        link_set, _, _, _, _, _, _, _ = p3d.threeD_robot_arm_links(link_vectors, alpha[:, i], joint_axes)
        l = threeD_update_links(l, link_set)
        
        # Redraw the updated links
        plt.pause(0.01)  # Draw and pause for animation effect

        # Save the current link set for grading
        link_set_history.append(link_set)
    
    
    plt.show()
    # Return values
    return (link_vectors, joint_axes, shape_to_draw, J, joint_velocity, T, a_start, sol, alpha, ax, link_colors, link_set, l, p, l_trace, link_set_history)
