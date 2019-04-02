# Execute this script in Blender (with Blensor plugin)

import bpy
from bpy import data as D
from bpy import context as C
import bmesh

from mathutils import *
from math import *
import blensor

import numpy as np

import os
import subprocess

from math import pi

def random_3D_unit_vector():
    """
    Generates a random 3D unit vector (direction) with a uniform spherical distribution
    Algo from http://stackoverflow.com/questions/5408276/python-uniform-spherical-distribution
    :return:
    """
    phi = np.random.uniform(0,np.pi*2)
    costheta = np.random.uniform(-1,1)

    theta = np.arccos( costheta )
    x = np.sin( theta) * np.cos( phi )
    y = np.sin( theta) * np.sin( phi )
    z = np.cos( theta )
    return [x,y,z]

def colored_random_3D_unit_vector(phi_limits = [0,np.pi*2], theta_limits = [np.pi, 0]):
    """
    Generates a random 3D unit vector (direction) with a uniform spherical distribution
    Algo from http://stackoverflow.com/questions/5408276/python-uniform-spherical-distribution
    :return:
    """
    
    if (len(phi_limits) == 0):
        phi = np.random.uniform(0,np.pi*2)
    else:
        phi_sets = len(phi_limits)//2
        random_phi_index = np.random.randint(phi_sets)*2
        phi = np.random.uniform(phi_limits[random_phi_index],phi_limits[random_phi_index+1])
    
    if (len(theta_limits) == 0):
        costheta = np.random.uniform(-1,1)
    else:
        theta_sets = len(theta_limits)//2
        random_theta_index = np.random.randint(theta_sets)*2
        costheta = np.random.uniform(np.cos(theta_limits[random_theta_index]),np.cos(theta_limits[random_theta_index+1]))

    theta = np.arccos( costheta )
    x = np.sin( theta) * np.cos( phi )
    y = np.sin( theta) * np.sin( phi )
    z = np.cos( theta )
    return [x,y,z]

def reset_blend():
#    bpy.ops.wm.read_factory_settings()

    for scene in bpy.data.scenes:
        for obj in scene.objects:
            scene.objects.unlink(obj)

    for bpy_data_iter in (
            bpy.data.objects,
            bpy.data.meshes,
            bpy.data.lamps,
            bpy.data.cameras,
    ):
        for id_data in bpy_data_iter:
            bpy_data_iter.remove(id_data)


def create_camera(name = 'Camera', position = [0.0, 0.0, 0.0], rotation = [0.0, 0.0, 0.0]):
    bpy.ops.object.camera_add(view_align=True, enter_editmode=False, 
                          location=(position[0], position[1], position[2]), 
                          rotation=(rotation[0], rotation[1], rotation[2]))
    bpy.context.object.name = name
    bpy.context.object.data.name = name


def norm_3d(v):
    return (v / np.sqrt((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]))).tolist()
        
        
def generate_view(camera_position, pc_resolution = 100, pcd_file = "/tmp/scan_test.pcd"):
    #compute camera orientation
    camera_position_versor = norm_3d(camera_position)
    axis = np.cross([0,0,1], camera_position_versor)
    axis = norm_3d(axis)
    theta = np.arccos(np.dot([0,0,1], camera_position_versor))
    T = Matrix()
    T = T.Rotation(theta, 4, axis)
    (x_rot, y_rot, z_rot) = T.to_euler()
    T[0][3] = camera_position[0]
    T[1][3] = camera_position[1]
    T[2][3] = camera_position[2]
    
    #Create camera object
    create_camera(name = 'Camera', position = camera_position, rotation = [x_rot, y_rot, z_rot])
    camera_obj = bpy.data.objects["Camera"]
    bpy.context.scene.camera = camera_obj
    
    #Generate pointcloud
    blensor.tof.scan_advanced(max_distance = 10, 
                              evd_file = pcd_file, 
                              add_blender_mesh = False, 
                              add_noisy_blender_mesh = False, 
                              tof_res_x = pc_resolution, 
                              tof_res_y = pc_resolution, 
                              lens_angle_w=33.6, 
                              lens_angle_h=33.6, 
                              flength = 10.0, 
                              evd_last_scan=True, 
                              noise_mu = 0.0, 
                              noise_sigma = 0.0, 
                              timestamp = 0.0, 
                              backfolding=False, 
                              world_transformation = T)
    #Reset camera                  
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.select_camera()                          
    bpy.ops.object.delete(use_global=False)
    
    #Export view information
    return [pcd_file, camera_position, theta, axis]               


def import_object(object_position, obj_file, obj_name = "Mesh"):
    
    filename, file_extension = os.path.splitext(obj_file)
    
    bpy.ops.object.select_all(action='DESELECT')
    
    if file_extension == '.obj':
        bpy.ops.import_scene.obj(filepath = obj_file,
                                axis_forward='X', 
                                axis_up='Z')
    elif file_extension == '.stl':
        bpy.ops.import_mesh.stl(filepath = obj_file, 
                                axis_up='Z', 
                                axis_forward='X')                         
                             
    bpy.ops.object.origin_set(center = 'BOUNDS')
    bpy.ops.transform.translate(value=(object_position[0], object_position[1], object_position[2]))
    #bpy.ops.transform.resize(value=(1.0, 1.0, 1.0))
    
    mat = bpy.data.materials.new(name="obj_material")
    bpy.context.selected_objects[0].data.materials.append(mat)
    bpy.context.selected_objects[0].active_material.diffuse_color = (1, 0, 0)
    
    bpy.context.selected_objects[0].name = obj_name
    
    bpy.ops.object.select_all(action='DESELECT')


def export_scene(file_name):
    bpy.ops.export_scene.obj(filepath=file_name, 
                             check_existing=True, 
                             axis_forward='X', 
                             axis_up='Z')


def loop(camera_distance_from_object, random_view_N, pc_resolution, pcd_file_prefix):
    view_infos = [] 
    for i in range(random_view_N):
        print("VIEW[",i+1,"]----------------------------------------------------------")
        bpy.ops.object.select_all(action='DESELECT')
        camera_versor = colored_random_3D_unit_vector(phi_limits = X_Y_angle_limits, theta_limits = Z_angle_limits)
        #theta = np.random.uniform(0,np.pi*2)
        #print("(x,w,z,theta): ", x, y, z, theta)    
        camera_position = [x * camera_distance_from_object for x in camera_versor]
        print("camera position: ", camera_position[0], camera_position[1], camera_position[2])
        
        pcd_file = pcd_file_prefix + str(i+1) + ".pcd"
        print("pcd_file: " + pcd_file)
        
        view_infos.append(generate_view(camera_position, pc_resolution, pcd_file))
    return view_infos
       
               
def visualize_view_infos(view_infos, random_view_N = 1):
    bpy.ops.object.select_all(action='DESELECT')
    for i in range(random_view_N):
        bpy.ops.mesh.primitive_cone_add(radius1=0.1, radius2=0, depth=0.2, view_align=False, enter_editmode=False, location = view_infos[i][1])
        bpy.ops.transform.rotate(value = view_infos[i][2], 
                                 axis = view_infos[i][3])
        
        view_name = "view_" + str(i+1)
        bpy.context.selected_objects[0].name = view_name

   
def test():
    bpy.ops.mesh.primitive_plane_add(radius=0.29,location=(0, 0, 0))
    bpy.ops.mesh.primitive_cone_add(radius1=0.1, radius2=0, depth=0.1, location=(0, 0, 0.05))
    mat = bpy.data.materials.new(name="MaterialName") #set new material to variable
    bpy.context.selected_objects[0].data.materials.append(mat) #add the material to the object
    bpy.context.object.active_material.diffuse_color = (1, 0, 0) #change color

    view_info = generate_view([0.5,0.0,0.5], 100, pcd_file = "/tmp/scan_1.pcd")
    print(view_info)
    bpy.ops.mesh.primitive_cone_add(radius1=0.1, radius2=0, depth=0.2, view_align=False, enter_editmode=False, location = camera_position)
    bpy.ops.transform.rotate(value = view_info[2], 
                             axis = view_info[3])
    bpy.context.selected_objects[0].name = "view_n"
    
    
reset_blend()

bpy.ops.object.lamp_add(type='POINT', view_align=False, location=(1, 1, 1))
bpy.ops.object.lamp_add(type='POINT', view_align=False, location=(1, -1, 1))
bpy.ops.object.lamp_add(type='POINT', view_align=False, location=(-1, 1, 1))
bpy.ops.object.lamp_add(type='POINT', view_align=False, location=(-1, -1, 1))
bpy.ops.object.select_all(action='DESELECT')

#Test script that generate a single view of the object
#test()

object_position = [0.0, 0.0, 0.0]
camera_distance_from_object = 0.5
random_view_N = 20
pc_resolution = 300
ground_through_leaf_size = 0.001

X_Y_angle_limits = []#[0,np.pi/2]
Z_angle_limits = [np.pi, np.pi/2+0.4 , np.pi/2-0.4, 0]

pcd_file_prefix = "/tmp/scan_"
obj_file = "/home/luca/CADs/soda_cans/330_can.obj"
#obj_file = "/home/luca/CADs/sockets/7174K640/7174K640_SOCKET.stl"

object_name = "Mesh"
import_object(object_position, obj_file, object_name)
ground_through_obj_file = pcd_file_prefix + "gt.obj"
export_scene(ground_through_obj_file)

views_info = loop(camera_distance_from_object, random_view_N, pc_resolution, pcd_file_prefix)
visualize_view_infos(views_info, random_view_N)

ground_through_pcd_file = pcd_file_prefix + "gt.pcd"
pcl_cmd = "pcl_mesh_sampling " + ground_through_obj_file + " " + ground_through_pcd_file + " -leaf_size " + str(ground_through_leaf_size)
subprocess.Popen(pcl_cmd, shell=True, stderr=subprocess.PIPE)

with open('/tmp/read_me.txt', 'w') as f:
    f.write("obj_file: %s \n" % obj_file)
    
    f.write("object_position: [")
    for item in object_position:
        f.write("%s " % item)
    f.write("]\n") 
    
    f.write("camera_distance_from_object: %s \n" % camera_distance_from_object)
    
    f.write("X_Y_angle_limits: [")
    for item in X_Y_angle_limits:
        f.write("%s " % item)
    f.write("]\n")
    
    f.write("Z_angle_limits: [")    
    for item in Z_angle_limits:
        f.write("%s" % item)
    f.write("]\n")
    
    f.write("views_resolution: %s \n" % pc_resolution)

    f.write("gt_leaf_size: %s \n" % ground_through_leaf_size)    

    f.write("random_view_N: %s \n" % random_view_N)

# to inspect the pointclouds:
# pcl_viewer scan_gt.pcd -use_point_picking