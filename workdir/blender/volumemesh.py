import gmsh
import pygmsh
import meshio
import os

with pygmsh.occ.Geometry() as geom:
    gmsh.option.setNumber("Mesh.CharacteristicLengthMin", 0.0001)
    gmsh.option.setNumber("Mesh.CharacteristicLengthMax", 10000)
    gmsh.option.setNumber("Mesh.Optimize", 1)
    gmsh.option.setNumber("Mesh.QualityType", 2)
    gmsh.merge("stl_exports/material_block.stl")
    n = gmsh.model.getDimension()
    s = gmsh.model.getEntities(n)
    lo = gmsh.model.geo.addSurfaceLoop([s[i][1] for i in range(len(s))])
    gmsh.model.geo.addVolume([lo])
    mesh = geom.generate_mesh()

destination_path = '/home/sofauser/workdir/blender/stl_exports/'
directory = os.path.dirname(destination_path)
meshio.write(directory + "/material_block.vtk",mesh,file_format="vtk",binary=False)
