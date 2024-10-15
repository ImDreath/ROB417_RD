import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import draw_3d_arm as draw_3d

def threeD_robot_arm_links(link_vectors, joint_angles, joint_axes):
    """
    Compute the 3D robot arm link endpoints given link vectors, joint angles, and joint axes.

    Parameters:
    link_vectors : list of np.ndarray
        A 1D list where each element is a 3x1 vector describing the vector from the base to the end of a link.

    joint_angles : np.ndarray
        A 1D array of joint angles.

    joint_axes : list of str
        A list where each entry is 'x', 'y', or 'z', designating the axis of the corresponding joint.

    Returns:
    link_set : list of np.ndarray
        A list of 3x2 matrices, where each matrix represents the start and endpoints of a link in the world frame.

    Additional Outputs:
    R_joints : list of np.ndarray
        The rotation matrices associated with the joints.

    R_links : list of np.ndarray
        The rotation matrices for the link orientations.

    link_set_local : list of np.ndarray
        The link vectors augmented with a zero vector representing the start of the link.

    link_vectors_in_world : list of np.ndarray
        The link vectors rotated by the link rotation matrices.

    links_in_world : list of np.ndarray
        The link start-and-end matrices rotated by the rotation matrices for the links.

    link_end_set : list of np.ndarray
        The endpoints of the links after taking the cumulative sum of the link vectors.

    link_end_set_with_base : list of np.ndarray
        The endpoints of the links including the origin (base) point.
    """

    # Step 1: Generate rotation matrices for the joints
    R_joints = draw_3d.threeD_rotation_set(joint_angles, joint_axes)

    # Step 2: Compute the cumulative product of the rotation matrices for the link orientations
    R_links = draw_3d.rotation_set_cumulative_product(R_joints)

    # Step 3: Build link start-and-end matrices with a zero vector representing the start of each link
    link_set_local = draw_3d.build_links(link_vectors)

    # Step 4: Rotate the link vectors by the link rotation matrices
    link_vectors_in_world = draw_3d.vector_set_rotate(link_vectors, R_links)

    # Step 5: Rotate the start-and-end matrices for each link
    links_in_world = draw_3d.vector_set_rotate(link_set_local, R_links)

    # Step 6: Compute the cumulative sum of the rotated link vectors to get the endpoints
    link_end_set = draw_3d.vector_set_cumulative_sum(link_vectors_in_world)

    # Step 7: Add a zero vector (origin) to the start of the link endpoints
    link_end_set_with_base = [np.zeros_like(link_end_set[0])] + link_end_set

    # Step 8: Generate the link matrices by adding the basepoint to the start-and-end matrices for each link
    link_set = draw_3d.place_links(links_in_world, link_end_set_with_base)

    return link_set, R_joints, R_links, link_set_local, link_vectors_in_world, links_in_world, link_end_set, link_end_set_with_base




def threeD_draw_links(link_set, link_colors, ax):
    """
    Draw a set of lines for a link structure into a specified axis.
    
    Inputs:
    link_set: A list of numpy arrays, each entry is a matrix whose
              columns are the endpoints of the lines describing one link
              of the system (as constructed by build_links with 3D input).
    link_colors: List of colors. Each entry can either be a standard matplotlib
                 color string (e.g., 'k' or 'r') or a 1x3 list/tuple of the RGB values
                 for the color (range from 0 to 1).
    ax: The handle to a 3D axis in which to plot the links.
    
    Output:
    l: A list of the same size as link_set, in which each entry is a 
       reference to the plotted line for that link.
    """
    
    # Initialize an empty list the same size as link_set
    l = []
    # print(link_set[0][0][0,:])



    # print(link_set[2][1].shape)
    # Loop through each link in the link_set and plot the corresponding line
    for i in range(len(link_set)):
        # Get the x, y, and z coordinates of the endpoints of the current link
        x_data = link_set[i][0, :]
        y_data = link_set[i][1, :]
        z_data = link_set[i][2, :]

        # Plot the line and save the handle
        line_handle, = ax.plot(x_data, y_data, z_data, linestyle='-', marker='o', color=link_colors[i])
        l.append(line_handle)

    return l
    
    return l
