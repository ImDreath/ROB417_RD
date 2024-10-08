import numpy as np
import matplotlib.pyplot as plt

def R_planar(theta):
    """
    Planar rotation matrix.

    Args:
    theta (float): Scalar value for rotation angle in radians.

    Returns:
    numpy.ndarray: 2x2 rotation matrix for rotation by theta.
    """
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])
    return R


def planar_rotation_set(joint_angles):
    #works
    # Generate a set of planar rotation matrices corresponding to the angles in the input vector

# Input:
    # joint_angles: a 1xn or nx1 vector of joint angles

# Output:
    # R_set: a cell array of the same size as the vector angles, in which
    #   each cell contains the planar rotation matrix for the angle in the
    #   corresponding entry of the vector

    # First, create an empty cell array called R_set that is the same size as the vector of joint angles
    sz = len(joint_angles)

    R_set = []
    # Loop over the joint angles, creating a rotation matrix and placing it in the corresponding entry of R_set
    for i in range(sz):
        R_set.append(R_planar(joint_angles[i]))

    # print(R_set)
    return R_set

def rotation_set_cumulative_product(R_set):
    #works
    """
    Take the cumulative product of a set of rotation matrices

    Input:
        R_set: A list, each element of which is a 2x2 or 3x3 rotation matrix

    Output:
        R_set_c: A list, the ith element of which is a 2x2 or 3x3 rotation matrix 
                  that is the cumulative product of the rotation matrices in 
                  R_set from the first matrix up to the ith matrix
    """
    # Start by copying R_set into a new variable R_set_c
    R_set_c = R_set
    sz = len(R_set)

    # Loop over R_set_c, multiplying each matrix into the one after it
    for i in range(1, sz):
        R_set_c[i] = R_set_c[i-1] @ R_set_c[i]

    return R_set_c

def vector_set_rotate(v_set, R_set):
    #works
    """
    Rotate a set of vectors specified in local coordinates by a set of rotation matrices.
    
    Parameters:
    v_set: list of numpy arrays, each of shape (2, 1) or (3, 1), representing vectors in local frames
    R_set: list of numpy arrays, each of shape (2, 2) or (3, 3), representing the rotation matrices
    
    Returns:
    v_set_R: list of numpy arrays, each of shape (2, 1) or (3, 1), the vectors rotated into the world frame
    """
    
    # Copy the v_set into a new list v_set_R
    v_set_R = v_set.copy()
    
    # Number of vectors
    sz = len(v_set)
    
    # Loop over each vector and apply the corresponding rotation matrix
    for i in range(sz):
        v_set_R[i] = R_set[i] @ v_set[i]
        # v_set_R[i] = np.dot(R_set[i], v_set[i])
        # print("print: " + str(R_set[i] @ v_set[i]))
    
    return v_set_R




def vector_set_cumulative_sum(v_set):
    ####
    """
    Take the cumulative sum of a set of vectors.

    Inputs:
        v_set: a list, each element of which is a 2-element or 3-element list

    Output:
        v_set_s: a list, each element of which is a 2-element or 3-element list
            and is the cumulative sum of vectors from v_set
    """
    
    # Start by copying v_set into a new variable v_set_s
    v_set_s = v_set.copy()
    sz = len(v_set)
    
    # Loop over v_set_s, adding each vector to the next vector    
    for i in range(1, sz):
        v_set_s[i] = v_set_s[i-1] + v_set_s[i]
    
    return v_set_s


def planar_robot_arm_endpoints(link_vectors, joint_angles):
    
    '''
    % Take a set of link vectors and joint angles, and return a matrix whose
% columns are the endpoints of all of the links (including the point that
% is the first end of the first link, which should be placed at the
% origin).
%
% Inputs:
%
%   link_vectors: a 1xn cell array, each element of which is a 2x1 vector
%       describing the vector from the base of the corresponding link to
%       its end
%   joint_angles: a nx1 vector, each element of which is the joint angle
%       preceeding the corresponding link
%   
% Outputs:
%
%   link_ends: a 3x(n+1) matrix, whose first column is the location
%       of the base of the first link (which should be at the origin), and
%       whose remaining columns are the endpoints of the links
%
% Additional outputs (These are intermediate variables. Having the option
%   to return them as outputs lets our automatic code checker tell you
%   where problems are in your code):
%
%   R_joints: The rotation matrices associated with the joints
%   R_links: The rotation matrices for the link orientations
%   link_vectors_in_world: The link vectors in their current orientations
%   link_end_set: The endpoints of the links after taking the cumulative
%       sum of link vectors
    '''
    # First, generate a list named 'R_joints' that contains a set of rotation matrices corresponding to the joint angles
    R_joints = planar_rotation_set(joint_angles)
    
    # Second, generate a list named 'R_links' that contains the orientations of the link frames by taking the cumulative products of the joint rotation matrices
    R_links = rotation_set_cumulative_product(R_joints)
    
    # Third, generate a list named 'link_vectors_in_world' that contains the link vectors rotated by the rotation matrices for the links
    link_vectors_in_world = vector_set_rotate(link_vectors, R_links)
    
    # Fourth, generate a list named 'link_end_set' that contains the endpoints of each link, found by taking the cumulative sum of the link vectors
    link_end_set = vector_set_cumulative_sum(link_vectors_in_world)
    
    # Fifth, add a zero vector (for the origin point at the base of the first link) to the beginning of link_end_set
    # link_end_set_with_base = [np.array([0, 0])] + link_end_set
    # Assuming link_end_set is already defined and is a list of NumPy arrays

    # Length of link_end_set + 1
    L = len(link_end_set) + 1

    # Initialize the link_end_set_with_base list with the base [0, 0]
    link_end_set_with_base = [np.array([[0], [0]])]  # First element is the base

    # Add the elements from link_end_set to link_end_set_with_base
    for i in range(1, L):
        link_end_set_with_base.append(link_end_set[i - 1])

    # Convert the set of link vectors to a simple matrix using numpy's hstack
    link_ends = np.hstack(link_end_set_with_base)
    
    return link_ends, R_joints, R_links, link_vectors_in_world, link_end_set, link_end_set_with_base



def create_axes(fignum):
    """
    Clear out a specified figure and create a clean set of axes in that figure
    with equal-axis aspect ratio.
    
    Parameters:
    fignum: The number of the figure (or a figure handle) in which to create the axes.
    
    Returns:
    ax: The created axes object.
    f: The figure object.
    """
    # Use plt.figure() to make sure the figure exists and get a figure handle
    f = plt.figure(fignum)
    
    # Use clf() to clear out any existing contents in the figure
    plt.clf()
    
    # Use plt.axes() to create axes in the figure
    ax = f.add_axes([0.1, 0.1, 0.7, 0.7])
    
    # Set the axes to have equal scaling
    ax.set_aspect('equal')
    
    # Set the box to be visible
    ax.spines['top'].set_visible(True)
    ax.spines['bottom'].set_visible(True)
    ax.spines['left'].set_visible(True)
    ax.spines['right'].set_visible(True)
    
    plt.show()
    return ax, f



def ME317_Assignment_draw_planar_arm():
    """
    Draws the planar arm as one line with circles at the endpoints.
    
    Returns:
        link_vectors: List of 2D vectors representing the links.
        joint_angles: Joint angles in radians.
        link_ends: Endpoints of the links.
        ax: Axes handle for the plot.
        l: Line object for the arm plot.
    """
    # Specify link vectors as a list of 2x1 NumPy arrays (equivalent to 1x3 cell array of 2x1 vectors in MATLAB)
    link_vectors = [np.array([[1], [0]]), np.array([[1], [0]]), np.array([[0.5], [0]])]
    
    # Specify joint angles as a 3x1 vector
    a = (2/5) * np.pi
    b = (-1/2) * np.pi
    c = (1/4) * np.pi
    joint_angles = np.array([a, b, c])
    
    # Get the endpoints of the links using the planar_robot_arm_endpoints function
    link_ends = planar_robot_arm_endpoints(link_vectors, joint_angles)
    
    print(link_ends)
    # Create figure and axes for the plot
    fignum = 317
    ax, _ = create_axes(fignum)
    
    

    # Draw a line from the data with circles at the endpoints
    l, = ax.lines('xdada', link_ends[0, :], 'ydata', link_ends[1, :], linestyle='-', marker='o')
    
    return link_vectors, joint_angles, link_ends, ax, l






