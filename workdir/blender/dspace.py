import bpy
import os


# option 1 from sketch: grid of cubes, tetrahedra or other mesh elements
 #def discretedesign():
    # TODO


# option 2,3,4 from sketch: cutting spherical bubbles into slab. Not included: air inlet tubes.
def swisscheese (dims,voids,res):
    ## input: dims - dimensions of slab in xyz (list), 
        # bd - list of sphere parameters, (...(x_center, y_center, z_center, radius)_i...)
        # res - "resolution" of sphere mesh, in terms of number of subdivisions. higher res-->finer mesh   

    # create a material block of dimensions w, d, h
    bpy.ops.mesh.primitive_cube_add(size=1, align='WORLD', location=(0, 0, 0), scale=(1,1,1)) 
    # leave scale as 1,1,1 here: changing scale here would scale it down and be incorrect

    mBlock = bpy.context.active_object
    mBlock.scale=(dims[0], dims[1], dims[2]) # change scale here to scale dim's up
    mBlock.name = 'material_block' #change name to identify it as material rather than void
    
    # iteratively draw spheres and punch away material from the bigger block
    for i in voids:
        # make the ith sphere
        bpy.ops.mesh.primitive_ico_sphere_add(subdivisions = res, radius=i[3], align='WORLD', location=(i[0],i[1], i[2]), scale=(1, 1, 1))
        sph = bpy.context.active_object
        
        # define and execute "difference" modifier which will remove material from the block
        mod = mBlock.modifiers.new("Boolean", type='BOOLEAN')
        mod.operation = 'DIFFERENCE'
        mod.object = sph
        bpy.context.view_layer.objects.active = mBlock
        bpy.ops.object.modifier_apply(modifier=mod.name)
        

def main():
    
    # TODO: Logic or notes here to decide block size, void dimensions, resolution, units etc
    matlDims = [2,1,1]
    dVoids = [[0.8, 0.4, 0.4, 0.4],[-0.5, -0.5, -0.5, 0.1],[0.2, -0.32, 0.1, 0.3],[-0.3, 0.2, -0.1, 0.4]]
    meshRes = 6;
    
    # generate meshes for the "swiss cheese"-style design (block of materials with holes)
    swisscheese(matlDims,dVoids,meshRes)

    # batch export each object mesh to separate stl file in chosen directory. all will be surface meshes
    destination_path = '/home/sofauser/workdir/blender/stl_exports/'
    directory = os.path.dirname(destination_path)
    bpy.ops.export_mesh.stl(filepath=destination_path, 
            use_scene_unit=True, batch_mode='OBJECT')

if __name__ == "__main__":
    main()
