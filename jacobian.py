import numpy as np
import matplotlib.pyplot as plt
import draw_3d_arm as d3d
import place_3D as p3d
from sympy import Matrix


def vector_set_difference(v, v_set):
#  Find the vector difference v - v_set (the vector pointing to v from each
#  element of v_set).
# 
#  Inputs:
# 
#    v: a vector
# 
#    v_set: a 1xn cell array of vectors, each of which is the same size as v
# 
#  Output
# 
#    v_diff: a 1xn cell array of vectors, each of which is the difference
#        between v and the corresponding vector in v_set

##########
#     Start by copying v_set into a new variable v_diff;
    
    v_diff = v_set
    
    # Loop over v_diff, subtracting each vector from v
   
    for i in range(len(v)):
            v_diff[i] = v - v_set[i]
        
    return v_diff




def draw_vectors_at_point(p, V, ax):
    """
    Draw the columns of V as arrows based at point p, in the specified axis.
    
    Inputs:
    p: A 3x1 numpy array designating the location of the vectors.
    V: A 3xn numpy array, where each column is a 3x1 vector to be drawn at point p.
    ax: The axis in which to draw the vectors (from matplotlib).
    
    Output:
    q: A list of handles to the quiver objects for the drawn arrows.
    """
    
    # First, hold the current plot in 'ax' to not overwrite existing drawings
    # ax.hold(True)  # Note: In modern Matplotlib, `ax.hold(True)` is deprecated and by default, multiple plots are retained.

    # Create an empty list named 'q' to store the quiver objects
    q = []

    # Loop over the columns of V
    for i in range(V.shape[1]):
        # Use ax.quiver to plot an arrow at point 'p', with vector components from the ith column of V
        quiv = ax.quiver(p[0], p[1], p[2], V[0][i], V[1][i], V[2][i])
        q.append(quiv)
    
    # Return the axis to its normal behavior (No need to explicitly hold(False) in matplotlib)
    return q



import numpy as np
from sympy import Matrix, zeros, symbols

def arm_Jacobian(link_vectors, joint_angles, joint_axes, link_number):
    # Use 'threeD_robot_arm_endpoints' to get 'link_ends', 'R_links', 'link_end_set' and 'link_end_set_with_base'
    # print(link_vectors)

    link_ends, R_joints, R_links, link_vectors_in_world, link_end_set, link_end_set_with_base = d3d.threeD_robot_arm_endpoints(link_vectors, joint_angles, joint_axes)
    
    # Use 'vector_set_difference' to get the vector to the end of link 'link_number' from each point in link_end_set_with_base

    v_diff = vector_set_difference(link_end_set_with_base[link_number], link_end_set_with_base)
    # v_diff = vector_set_difference(link_end_set_with_base[-1], link_end_set_with_base)

    # Use 'threeD_joint_axis_set' to turn the joint axes into a set of axis vectors called 'joint_axis_vectors'
    joint_axis_vectors = p3d.threeD_joint_axis_set(joint_axes)

    # Use 'vector_set_rotate' to rotate the joint axes by the link orientations
    joint_axis_vectors_R = d3d.vector_set_rotate(joint_axis_vectors, R_links)

    # Create a zero matrix to hold the Jacobian. It should have three rows, and as many columns as there are joints
    J = np.zeros((3, len(joint_angles)))

    # Check if joint_axis_vectors_R or v_diff contain any symbolic variables
    is_symbolic = any(isinstance(vec, Matrix) for vec in joint_axis_vectors_R) or any(isinstance(vec, Matrix) for vec in v_diff)

    # If symbolic, convert J to symbolic matrix
    if is_symbolic:
        J = Matrix.zeros(3, len(joint_angles))

    

    # Fill in the columns of the Jacobian
    for i in range(link_number):
        if isinstance(joint_axis_vectors_R[i], Matrix):  # This is for symbolic computations
            J[:, i] = joint_axis_vectors_R[i].cross(v_diff[i])
        else:
            # print(f"len(joint_axis_vectors_R{i} is {len(joint_axis_vectors_R[i])}\n")
            # print(f"joint_axis_vectors_R{i} is: \n {joint_axis_vectors_R[i]}\n")
            JAV = joint_axis_vectors_R[i].reshape(3)
            # print(f"JAV is: \n {JAV}\n")
            
            # print(f"len(v_diff is {len(v_diff[i])}\n")
            # print(f"v_diff is \n{v_diff[i]}\n")

            J[i] = np.cross(joint_axis_vectors_R[i].reshape(3), v_diff[i].reshape(3))


    return J, link_ends, link_end_set, link_end_set_with_base, v_diff, joint_axis_vectors, joint_axis_vectors_R, R_links


def create_subaxes(fignum,m,n,p):
    '''
    # % Clear out a specified figure and create a clean set of axes in that
    # % figure with equal-axis aspect ratio
    # %
    # % Input:
    # %
    # %   fignum: The number of the figure (or a figure handle) in which to
    # %       create the axes 
    # %   m: the number of rows of subplots to create
    # %   n: the number of columns of subplots to create
    # %   p: the number of subplots to create (should be less than or equal to
    # %       m*n)
    # %
    # % Outputs:
    # %
    # %   ax: A cell array of handles to the created subplot axes
    # %   f: A handle to the figure that was created
    '''
    #     %%%%%%%
    #     % Use the 'figure' command to make sure that the figure in the input
    #     % exists. The output to figure provides a handle to the figure
        
    #     f = figure(fignum);
    f = plt.figure(fignum)
        
    #     %%%%%%%
    #     % Use the 'clf' command with the 'reset' option to clean out any
    #     % existing contents in the figure. Use the figure handle to make sure
    #     % that the 'clf' command targets this figure
        
    #     clf(f, 'reset');
        
    #     % Create an empty 1xp cell named ax to hold the axis handles
    #     ax = cell(1,p);
    ax = [None] * p
    #     %%%%%%%
    #     % Loop over the number of subplots requested
        
    #     for i = 1:p
    for i in range(p):
        
            
    #         %%%%%%%
    #         % Use the 'subplot' command to create the (idx)th subplot axis for
    #         % plotting. Use the 'Parent' option with the figure handle to make
    #         % sure that the axes are created in that figure. The output of
    #         % 'subplot' provides a handle to the axis. Store this this handle
    #         % as 'ax{idx}'
    
    #         ax{i} = subplot(m,n,i,'Parent', f)
        ax[i] = f.add_subplot(m,n,i+1, projection='3d')

    #         %%%%%%
    #         % Use the 'axis' command with the 'equal' option to make sure that all
    #         % the axes of the suplot have the same length scale. Use the handle of
    #         % the axis to make sure that this action applies to the axis you want
    #         % it to apply to

    #         axis('equal')

    #         %%%%%%
    #         % Use the 'box' command with the 'on' option to have all edges of the
    #         % plotting axis marked Use the handle of the axis to make sure that
    #         % this action applies to the axis you want it to apply to
            
    #         box('on')
    return [ax,f]



def ME317_Assignment_draw_3D_arm_with_Jacobian():
    """
    Creates a set of plots illustrating the relationship between the geometry of the arm
    and the Jacobians of the links.
    
    Returns:
    Tuple containing link vectors, joint angles, joint axes, Jacobians, link ends, link end sets,
    axis handles, line handles, quiver handles, and others.
    """
    # Define link vectors as a list of 3x1 numpy arrays
    link_vectors = [np.array([[1], [0], [0]]), np.array([[1], [0], [0]]), np.array([[0], [0], [0.5]])]
    
    # Define joint angles as a 3x1 numpy array
    joint_angles = np.array([2 * np.pi / 5, -np.pi / 4, np.pi / 4])
    
    # Define joint axes
    joint_axes = ['z', 'y', 'x']
    
    # Initialize a list to store the Jacobians for each link
    J = [None] * len(link_vectors)
    
    # Loop to calculate Jacobians for each link
    for i in range(len(J)):
        (J[i], link_ends, link_end_set, link_end_set_with_base, 
         v_diff, joint_axis_vectors, joint_axis_vectors_R, R_links) = arm_Jacobian(link_vectors, joint_angles, joint_axes, i+1)
    
    # Plotting
    p = len(link_vectors)
    m = int(np.ceil(np.sqrt(p)))
    n = m
    fignum = 317
    ax, f = create_subaxes(fignum, m, n, p)
    
    # Initialize empty lists to store line and quiver handles
    l = [None] * len(J)
    l2 = [None] * len(J)
    l3 = [None] * len(J)
    q = [None] * len(J)
    
    # Plot the robot arm links in each subplot
    for i in range(len(ax)):
        l[i] = ax[i].plot(link_ends[0, :], link_ends[1, :], link_ends[2, :], marker='o')[0]
    

    
    # Add arrows for the columns of the corresponding Jacobian
    for i in range(len(ax)):
        p = link_end_set[i]
        q[i] = draw_vectors_at_point(p, J[i], ax[i])
    
    
    # Add lines and dashed lines between link ends and joints for each Jacobian
    for i in range(len(l2)):
        l2[i] = [None] * len(J[i])
        
        for j in range(i + 1):
            x = np.column_stack((link_end_set_with_base[j], link_end_set[i]))
            color = q[i][j].get_color()
            l2[i][j] = ax[i].plot(x[0, :], x[1, :], x[2, :], color=color, linestyle=':')[0]
        
        l3[i] = [None] * len(J[i])
        for j in range(i + 1):
            x = np.column_stack((link_ends[:, j], link_ends[:, j] + joint_axis_vectors_R[j]))
            color = q[i][j].get_color()
            l3[i][j] = ax[i].plot(x[0, :], x[1, :], x[2, :], color=color, linestyle='--')[0]
    plt.show()
    
    return link_vectors, joint_angles, joint_axes, J, link_ends, link_end_set, ax, l, l2, l3, q
