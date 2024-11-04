import numpy as np
import matplotlib.pyplot as plt

def Rx(psi):
    """
    Rotation matrix about the x-axis.

    Parameters:
    psi : float
        Scalar value for the rotation angle.

    Returns:
    R : np.ndarray
        3x3 rotation matrix for rotation by psi around the x-axis.
    """
    
    R = np.array([[1,           0,            0],
                  [0, np.cos(psi), -np.sin(psi)],
                  [0, np.sin(psi),  np.cos(psi)]])
    
    return R

def Ry(phi):
    """
    Rotation matrix about the y-axis.

    Parameters:
    psi : float
        Scalar value for the rotation angle.

    Returns:
    R : np.ndarray
        3x3 rotation matrix for rotation by psi around the y-axis.
    """
    
    R = np.array([[np.cos(phi),  0, np.sin(phi)],
                  [          0,  1,           0],
                  [-np.sin(phi), 0, np.cos(phi)]])

    return R

def Rz(theta):
    """
    Rotation matrix about the x-axis.

    Parameters:
    psi : float
        Scalar value for the rotation angle.

    Returns:
    R : np.ndarray
        3x3 rotation matrix for rotation by psi around the x-axis.
    """
     
    R = np.array([[np.cos(theta), -np.sin(theta), 0],
                  [np.sin(theta),  np.cos(theta), 0],
                  [            0,              0, 1]])

    
    return R


def threeD_rotation_set(joint_angles, joint_axes):
    """
    Generate a set of 3D rotation matrices corresponding to the angles and axes 
    in the input vector.

    Parameters:
    joint_angles : list or np.ndarray
        A 1D list or array of joint angles.

    joint_axes : list
        A list of the same size as joint_angles, where each element is a one-character 
        string ('x', 'y', or 'z') specifying which axis the rotation is around.

    Returns:
    R_set : list
        A list of the same size as the input vector, where each entry contains the 
        rotation matrix for the corresponding angle and axis.
    """
    
    # Initialize an empty list for storing rotation matrices
    R_set = [None] * len(joint_angles)
    # R_set = np.empty_like(joint_angles, float)
    
    # Loop over the joint angles and axes
    for i, axis in enumerate(joint_axes):
        # print("sdfd")
        if axis == 'x':
            # print(joint_angles[i])
            R_set[i] = Rx(joint_angles[i])
        elif axis == 'y':
            R_set[i] = Ry(joint_angles[i])
        elif axis == 'z':
            R_set[i] = Rz(joint_angles[i])
        else:
            raise ValueError(f"{axis} is not a known joint axis")

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


def threeD_robot_arm_endpoints(link_vectors, joint_angles, joint_axes):
    """
    Compute the endpoints of all links in a 3D robotic arm given the link vectors, joint angles, and joint axes.

    Parameters:
    link_vectors : list of np.ndarray
        A 1D list where each element is a 3x1 vector describing the vector from the base to the end of a link.

    joint_angles : np.ndarray
        A 1D array of joint angles.

    joint_axes : list of str
        A list where each entry is 'x', 'y', or 'z', designating the axis of the corresponding joint.

    Returns:
    link_ends : np.ndarray
        A 3x(n+1) matrix, where the first column is the base of the first link (at the origin), and the remaining 
        columns are the endpoints of the links.

    Additional Outputs:
    R_joints : list of np.ndarray
        The rotation matrices associated with the joints.

    R_links : list of np.ndarray
        The rotation matrices for the link orientations.

    link_vectors_in_world : list of np.ndarray
        The link vectors in their current orientations.

    link_end_set : list of np.ndarray
        The endpoints of the links after taking the cumulative sum of link vectors.
    
    link_end_set_with_base : list of np.ndarray
        The endpoints including the base (origin) point.
    """

    # Step 1: Generate a list of rotation matrices corresponding to the joint angles
    R_joints = threeD_rotation_set(joint_angles, joint_axes)
    
    # Step 2: Compute the cumulative product of the rotation matrices to get link orientations
    R_links = rotation_set_cumulative_product(R_joints)

    # Step 3: Rotate the link vectors by the link rotation matrices
    link_vectors_in_world = vector_set_rotate(link_vectors, R_links)

    # Step 4: Compute the cumulative sum of the rotated link vectors to get the endpoints of each link
    link_end_set = vector_set_cumulative_sum(link_vectors_in_world)

    # Step 5: Add a zero vector to the start of the link end set (representing the origin)
    # link_end_set_with_base = [np.array([0, 0, 0])] + link_end_set

    L = len(link_end_set) + 1
    link_end_set_with_base = [None] * L
    link_end_set_with_base[0] = np.array([[0], [0], [0]])
    for i in range(1,L):
        link_end_set_with_base[i] = link_end_set[i-1]

    # Step 6: Convert the set of link vectors to a matrix
    link_ends = np.column_stack(link_end_set_with_base)

    return link_ends, R_joints, R_links, link_vectors_in_world, link_end_set, link_end_set_with_base

def build_links(link_vectors):
    #works
    """
    Take a set of link vectors and augment each with a zero vector representing
    the base of the link.
    
    Parameters:
    link_vectors: list of numpy arrays, where each element is an m×1 vector representing the 
                  link vector in its local frame.
    
    Returns:
    link_set: list of numpy arrays, each element is an m×2 matrix, where the first column is
              all zeros and the second column is the link vector.
    """
    
    # Initialize an empty list with the same length as link_vectors
    link_set = [None] * len(link_vectors)

    # Loop over the vectors in link_vectors
    for i, link_vector in enumerate(link_vectors):
        # Construct a matrix where the first column is zeros and the second column is the link vector
        link_set[i] = np.hstack((np.zeros((link_vector.shape[0], 1)), link_vector))
    
    return link_set


def place_links(links_in_world, link_end_set_with_base):
    """
    Use the locations of the ends of a set of links to place the
    start-and-end matrices for the links.

    Inputs:
    links_in_world: a list of numpy arrays, each element of which is a
                    matrix whose columns are the start-and-end points
                    of a link in its rotated-but-not-translated frame.
    
    link_end_set_with_base: a list of numpy arrays, each element of which is
                            a vector containing the world location of the end 
                            of the corresponding link.
    
    Output:
    link_set: a list of numpy arrays, each element of which is a 2xn matrix
               whose columns are the start-and-end points of the link in its
               after the link has been placed in the world.
    """
    
    # Start by copying links_in_world into a new variable named 'link_set'
    link_set = links_in_world.copy()
    
    # Loop over link_set, adding each the location of the base of each
    # link to the link's endpoint matrix to generate the endpoint locations
    for i in range(len(links_in_world)):
        link_set[i] = link_set[i] + link_end_set_with_base[i]
        
    return link_set

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
    # plt.clf()
    
    # Use plt.axes() to create axes in the figure
    # ax = f.add_axes([0.1, 0.1, 0.7, 0.7])
    ax = f.add_subplot(111, projection='3d')
    
    # Set the axes to have equal scaling
    # ax.set_aspect('equal')
    
    # # Set the box to be visible
    # ax.spines['top'].set_visible(True)
    # ax.spines['bottom'].set_visible(True)
    # ax.spines['left'].set_visible(True)
    # ax.spines['right'].set_visible(True)
    # print(ax)
    # plt.show()
    return ax, f