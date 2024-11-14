import numpy as np
import jacobian as jc
import draw_3d_arm as d3a
import matplotlib.pyplot as plt
from sympy import symbols, pi, Matrix

# Tests functions from jacobian.py


#################################

# # Test vector_set_difference - works :)

# v_set = np.array([[[1],[0]], [[0],[1]]])
# v_single = np.array([[2],[2]])
# v_diff = jc.vector_set_difference(v_single,v_set)
# for i in v_diff:
#     print(i)


#################################

# Tests draw_vectors_at_point - works, view is different but plots are same

# p = np.array([[1], [1], [1]])
# V = np.array([[[1], [0], [0]], [[0], [1], [0]], [[0], [0], [1]]])
# print(V)
# ax, f = d3a.create_axes(317)
# q = jc.draw_vectors_at_point(p,V,ax)
# plt.show()

###########################

# # Test Arm_Jacobian

# # Assuming that the previous arm_Jacobian function exists and is correct.

# # First: Numeric Input
# link_vectors_numeric = np.array([[[1], [0], [0]], [[0.5], [0], [0]], [[0.5], [0], [0]]])
# joint_angles_numeric = np.array([0.4, -0.5, 0.25]) * np.pi
# joint_axes_numeric = ['z', 'z', 'z']
# link_number_numeric = 3

# # Run with numeric input
# J, link_ends, link_end_set, link_end_set_with_base, v_diff, joint_axis_vectors, joint_axis_vectors_R, R_links = jc.arm_Jacobian(
#     link_vectors_numeric, joint_angles_numeric, joint_axes_numeric, link_number_numeric
# )

# # Second: Symbolic Input
# a, b, c = symbols('a b c')
# joint_angles_symbolic = [a, b, c]
# joint_axes_symbolic = ['z', 'z', 'z']
# link_number_symbolic = 2

# # Run with symbolic input
# J_symbolic, link_end_set_with_base_symbolic, v_diff_symbolic, joint_axis_vectors_symbolic, joint_axis_vectors_R_symbolic, R_links_symbolic = jc.arm_Jacobian(
#     link_vectors_numeric, joint_angles_symbolic, joint_axes_symbolic, link_number_symbolic
# )

# # Output the numeric and symbolic Jacobians
# print("Numeric Jacobian:\n", J)
# print("Symbolic Jacobian:\n", J_symbolic)


##############################################################\

# # Test Create_subaxes

# fignum = 317
# [ax,f] = jc.create_subaxes(fignum,3,2,5)
# plt.show()

##################3

[link_vectors, joint_angles, joint_axes, J, link_ends, link_end_set, ax, l, l2, l3, q] = jc.ME317_Assignment_draw_3D_arm_with_Jacobian()
