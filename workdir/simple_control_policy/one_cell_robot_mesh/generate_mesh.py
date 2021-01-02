import os, sys
# in the mesh density test the block was 10x10x20 and we used 0.55 length scale, 



size = "{:<05}".format(0.55)
cmd_string = "gmsh -algo meshadapt -clscale "+size+" -1 -2 -3 -format msh22 -o volume.msh one_cell_volume.brep"
print(cmd_string)
os.system(cmd_string)

cmd_string = "gmsh -algo meshadapt -clscale "+size+" -1 -2 -format stl -o cavity_0_0_0.stl one_cell_1_1_1.brep"
print(cmd_string)
os.system(cmd_string)




