import os, sys
# in the mesh density test the block was 10x10x20 and we used 0.55 length scale, 



size = "{:<05}".format(0.55)
cmd_string = "gmsh -algo meshadapt -clscale "+size+" -1 -2 -3 -format msh22 -o two_cell_robot_solid.msh two_cell_robot_solid.brep"
print(cmd_string)
os.system(cmd_string)

cmd_string = "gmsh -algo meshadapt -clscale "+size+" -1 -2 -format stl -o two_cell_robot_cell1.stl two_cell_robot_cell1.brep"
print(cmd_string)
os.system(cmd_string)

cmd_string = "gmsh -algo meshadapt -clscale "+size+" -1 -2 -format stl -o two_cell_robot_cell2.stl two_cell_robot_cell2.brep"
print(cmd_string)
os.system(cmd_string)



