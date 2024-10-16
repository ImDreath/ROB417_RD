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
        # print(ax)
        # Plot the line and save the handle
        line_handle, = ax.plot(x_data, y_data, z_data, linestyle='-', marker='o', color=link_colors[i])
        l.append(line_handle)

    return l


def threeD_joint_axis_set(joint_axes):
    ''' Generate a set of unit vectors along specified x, y, or z axes

    Input:

    joint_axes: a cell array , each element of which is a 
       one-character string 'x','y', or 'z' that specifies
       an axis of rotation

    Output:

    joint_axis_vectors: a cell array of the same size as the vector
       joint_axes, in which each cell contains the 3x1 unit vector in the
       direction specified in the corresponding entry of joint_axes
    '''
    ########
    #  Start by creating an empty cell array of the same size as joint_axes,
    #  named 'joint_axis_vectors'
    
    joint_axis_vectors = [None] * len(joint_axes)

    ##########33
    # Loop over the joint axes
        
        ##########3
    ''' Use 'switch/case' to check which axis the joint is aligned with
         around. For 'x','y', or 'z', this should result in a unit vector
         along the appropriate axis.
        
         Any other string should trigger the 'otherwise' part of
         switch/case, in which there should be an 'error' function that
         tells the user what they did wrong. For example, 
        
         error([joint_axis ' is not a known joint axis'])
         
         would make the program stop, and tell the user what string was
         incorrectly supplied as the description of a unit vector.
        '''

    for i, axis in enumerate(joint_axes):
        
        if axis == 'x':
            joint_axis_vectors[i] = np.array([[1], [0], [0]])
        elif axis == 'y':
            joint_axis_vectors[i] = np.array([[0], [1], [0]])
        elif axis == 'z':
            joint_axis_vectors[i] = np.array([[0], [0], [1]])
        else:
            print(f"{joint_axes} is not a known joint axis")
    
    return joint_axis_vectors


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def ME317_Assignment_draw_3D_arm_individual_links():
    """
    Draw the arm as a set of lines, one per link.
    
    Returns:
    link_vectors: List of link vectors.
    joint_angles: Numpy array of joint angles.
    joint_axes: List of joint axes.
    link_colors: List of link colors.
    link_set: List of start-and-end points for the links.
    R_links: List of rotation matrices for the links.
    joint_axis_vectors: List of joint axis vectors.
    joint_axis_vectors_R: Rotated joint axis vectors.
    ax: Axis handle for the plot.
    l: Line handles for the links.
    l3: Line handles for the joint axis vectors.
    """
    
    # Specify link vectors as a list of 3x1 numpy arrays
    link_vectors = [np.array([[1], [0], [0]]), np.array([[1], [0], [0]]), np.array([[0], [0], [0.5]])]
    
    # Specify joint angles as a numpy array
    a = (2/5)*np.pi
    b = (-1/4)*np.pi
    c = (1/4)*np.pi
    joint_angles = np.array([a, b, c])
    
    # Specify joint axes as a list
    joint_axes = ['z', 'y', 'x']
    
    # Specify colors of links as a list
    link_colors = ['r', [0.5, 0.5, 0.5], 'b']
    
    # Generate link_set and R_links from the 'threeD_robot_arm_links' function
    (link_set,
    R_joints,
    R_links,
    link_set_local,
    link_vectors_in_world,
    links_in_world,
    link_end_set,
    link_end_set_with_base) = threeD_robot_arm_links(link_vectors, joint_angles, joint_axes)
    
    # Use 'threeD_joint_axis_set' to create joint axis vectors
    joint_axis_vectors = threeD_joint_axis_set(joint_axes)
    
    # Rotate the joint axis vectors using 'vector_set_rotate'
    joint_axis_vectors_R = draw_3d.vector_set_rotate(joint_axis_vectors, R_links)
    
    # Create figure and axes for the plot
    fignum = 317
    ax,f = draw_3d.create_axes(fignum)
    # print(ax)
    
    # Draw links and save the handles to the list 'l'
    l = threeD_draw_links(link_set, link_colors, ax)
    
    # Draw dashed lines for joint axis vectors and save handles in 'l3'
    l3 = []

    for i in range(len(link_set)):
        x_data = [link_set[i][0, 0], link_set[i][0, 0] + joint_axis_vectors_R[i][0][0]]
        print(link_set[i][0, 0])
        print(joint_axis_vectors_R[i][0][0])
        print(link_set[i][0, 0] + joint_axis_vectors_R[i][0])
        y_data = [link_set[i][1, 0], link_set[i][1, 0] + joint_axis_vectors_R[i][1][0]]
        z_data = [link_set[i][2, 0], link_set[i][2, 0] + joint_axis_vectors_R[i][2][0]]
        
        l3.append(ax.plot(x_data, y_data, z_data, linestyle=':', color=link_colors[i]))
    plt.show()