# Base File for Blender Dynamic Environment Generation
import bpy # You need Blender to run this.
import bmesh # for decimation.
import glob
import os
from morse_simulator.algorithms.config import config

ABS_PATH = os.path.abspath('./')

scene = bpy.context.scene

class sel:
    """Function Class for operating on SELECTED objects"""
    # Differential
    def translate(v):
        bpy.ops.transform.translate(
            value=v, constraint_axis=(True, True, True))
    # Differential
    def scale(v):
        bpy.ops.transform.resize(value=v, constraint_axis=(True, True, True))
    # Differential
    def rotate_x(v):
        bpy.ops.transform.rotate(value=v, axis=(1, 0, 0))
    # Differential
    def rotate_y(v):
        bpy.ops.transform.rotate(value=v, axis=(0, 1, 0))
    # Differential
    def rotate_z(v):
        bpy.ops.transform.rotate(value=v, axis=(0, 0, 1))

def decimateComplexObjects():
    THR = 0.85  # <-- the only parameter. The smaller, the stricter

    for element in bpy.data.objects:
        if element.type != "MESH": continue

        # reset
        for m in element.modifiers:
            if m.name == 'ScriptedDecimate':
                element.modifiers.remove(m)

        # scoring function
        vert = len(element.data.vertices)
        areabbox = (element.dimensions.x*element.dimensions.y
                   + element.dimensions.y*element.dimensions.z
                   + element.dimensions.x*element.dimensions.z)*2
        indicator = vert/areabbox
        
        # select and decimate!
        if (indicator > THR):
            print('[INFO] Decimating...', flush = True)
            element.select=True
            decimator = element.modifiers.new("ScriptedDecimate", type = "DECIMATE")
            decimator.ratio = 0.01 # (THR/indicator)**(1/9) # Very high decimation
            bpy.context.scene.update()
        else:
            element.select=False
    print('[INFO] Decimation Successful')

def createEnvs():
    # First we delete the default cube in blender.
    for o in bpy.context.scene.objects:
        if o.type == 'MESH':
            o.select = True
        else:
            o.select = False

    # Call the operator only once
    bpy.ops.object.delete()

    # Now we proceed with creating our sub-environment
    if not os.path.exists(config.savePath + '/logs/allObjs.log'):
        obj_path = r'/home/athrva/Desktop/allLabCode/obj_dataset'
        all_files = glob.glob(obj_path + os.path.sep + '*.obj')
        with open(config.savePath + '/logs/allObjs.log', 'w+') as objPaths:
            for file in all_files:
                #if i < 10:
                objPaths.write(file)
                objPaths.write('\n')
    else:
        with open(config.savePath + '/logs/allObjs.log', 'r') as objPaths:
            all_files = objPaths.readlines()
    # Total number of environments in the super-environment = r*c
    if not os.path.exists(config.savePath + '/logs/superEnvironmentIdx.log'):
        with open(config.savePath + '/logs/superEnvironmentIdx.log','w+') as sIdx:
            sIdx.write(str(0)) # Write the initial Index
    with open(config.savePath + '/logs/superEnvironmentIdx.log', 'r') as sIdx:
        b = int(sIdx.readlines()[0])
    r = config.num_robots_per_senv # Number of environments in the grid super-environment (Rows)
    c = 1 # Number of environments in the grid super-environment (Columns)
    x = [(i % c)*50 for i in range(r*c)] # X coordinate for translation
    y = [(i // c)*50 for i in range(r*c)] # Y coordinate for transaltion
    
    with open(config.savePath + '/logs/translation_coordinates.txt', 'w+') as translation_coordinates:
        for i in range(len(x)):
            translation_coordinates.write(str(x[i])+','+str(y[i]))
            translation_coordinates.write('\n')
    print('------------------------------------------------------------------------')
    idx = -1
    dimensions = [] # this stores all the map extents so that we can use them later.
    
    for i in range((len(x)-1)*b,1+((b+1)*(len(x)-1))): # Iterating over all environments
        idx += 1 # Index for the translation coordinates
        if all_files[i][-1] == '\n':
            all_files[i] = all_files[i][:-1]
        imported_object = bpy.ops.import_scene.obj(filepath=all_files[i], split_mode = 'OFF') # Import the environment
        obj_object = bpy.context.selected_objects[0] # Bring the imported environment into context
        obj_object.select = True
        sel.scale((0.21, 0.21, 0.21)) # Scale the environment (by default the envs are too large.)
        sel.rotate_x(-3.1415 / 2) # Rotation correction (The envs are rotated by default)
        sel.translate((x[idx]+3, y[idx]+3, 0)) # Translate the imported envs to put them in place. Notice the added offset
        x_dim,y_dim,z_dim = obj_object.dimensions[0], obj_object.dimensions[1], obj_object.dimensions[2]
        dimensions.append([x_dim, y_dim, z_dim])
        #lamp_data = bpy.data.lamps.new(name=f"New Lamp_{i}", type='HEMI') # Adding a light for visualization
        #lamp_data.energy = 0.01
        #lamp_object = bpy.data.objects.new(name=f"New Lamp object {i}", object_data=lamp_data)
        #scene.objects.link(lamp_object)
        #lamp_object.location = (x[idx], y[idx], 75.0) # Setting the position of the light to be exactly above the sub-environment.
        #lamp_object.select = True
        #scene.objects.active = lamp_object
        with open(config.savePath + f'/results/map_extent.txt', 'a+') as map_extent:
            map_extent.write(f'{x_dim},{y_dim},{z_dim}\n')
        
        with open(config.savePath + '/logs/allObjs.log',"r+") as f:
            new_f = f.readlines()
            f.seek(0)
            for line in new_f:
                if all_files[i] not in line:
                    f.write(line)
            f.truncate()
        
        with open(config.savePath + '/logs/allObjs.log', 'r') as objPaths:
            all_files = objPaths.readlines()
    # We have added all the required objects.
    """
    Basically, the objects are not optimized, i.e., the contain more vertices than required. 
    So we decimate the meshes to reduce complexity. This saves on the render time.
    """
    decimateComplexObjects()
    
    
    bpy.ops.wm.save_as_mainfile(filepath=ABS_PATH + f'/workingSuperEnvironment/superEnvironment_{b}.blend')
    b += 1 # Increment the SuperEnvironment Number.
    with open(config.savePath + '/logs/superEnvironmentIdx.log','w+') as sIdx:
        sIdx.write(str(b)) # Write the incrmented Index Number into log file.
    bpy.ops.wm.quit_blender()

if __name__ == '__main__':
    createEnvs()
