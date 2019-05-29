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

import mathutils
import math


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

def random_rotation_vector(vector, theta):
    
    # generate random axis perpendivular to vector:
    vector_ver = mathutils.Vector((vector[0], vector[1], vector[2]))
    vector_ver = vector_ver.normalized()
    #print("vector_ver: ", vector_ver) 
    
    [axis_x, axis_y, axis_z] = random_3D_unit_vector()
    axis = mathutils.Vector((axis_x, axis_y, axis_z))
    axis = vector_ver.cross(axis) # perpendibular to vector
    #print("axis: ", axis)
    
    # compute correspondint rot matrix
    mat_rot = mathutils.Matrix.Rotation(theta, 4, axis)
    #print("mat_rot: ", mat_rot)
    
    # rotate vector
    new_vector = mathutils.Vector((vector[0], vector[1], vector[2]))
    new_vector.rotate(mat_rot)
    #print("new_vector: ", new_vector)

    return [new_vector[0], new_vector[1], new_vector[2]]

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
                             axis_forward='Y', 
                             axis_up='Z')


def loop(camera_distance_from_object, angle_between_cameras, random_view_N, pc_resolution, pcd_file_prefix):
    view_infos = [] 
    for i in range(random_view_N):
        print("VIEW[",i+1,"]----------------------------------------------------------")
        bpy.ops.object.select_all(action='DESELECT')

        cam1_versor = random_3D_unit_vector()
        cam1_position = [x * camera_distance_from_object for x in cam1_versor]
                
        cam2_versor = random_rotation_vector(cam1_versor, angle_between_cameras)
        cam2_position = [x * camera_distance_from_object for x in cam2_versor]
        print("cam1_position: ", cam1_position[0], cam1_position[1], cam1_position[2])
        print("cam2_position: ", cam2_position[0], cam2_position[1], cam2_position[2])
                
        pcd1_file = pcd_file_prefix + str(i+1) + "a.pcd"
        pcd2_file = pcd_file_prefix + str(i+1) + "b.pcd"
        print("pcd1_file: " + pcd1_file)
        print("pcd2_file: " + pcd2_file)
        
        curr_view_1_info = generate_view(cam1_position, pc_resolution, pcd1_file)
        curr_view_2_info = generate_view(cam2_position, pc_resolution, pcd2_file)
        
        #CLEANING FILES
        pcd1_file_rename = pcd_file_prefix + str(i+1) + "a00000.pcd"
        pcd2_file_rename = pcd_file_prefix + str(i+1) + "b00000.pcd"
        os.rename(pcd1_file_rename, pcd1_file)
        os.rename(pcd2_file_rename, pcd2_file) 
        pcd1_file_remove = pcd_file_prefix + str(i+1) + "a_noisy00000.pcd"
        pcd2_file_remove = pcd_file_prefix + str(i+1) + "b_noisy00000.pcd" 
        os.remove(pcd1_file_remove)
        os.remove(pcd2_file_remove)      
              
        generatere_cfg_file(curr_view_1_info)
        generatere_cfg_file(curr_view_2_info)
        
        view_infos.append(curr_view_1_info)
        view_infos.append(curr_view_2_info) 
    return view_infos
       
               
def visualize_view_infos(view_infos, random_view_N = 1):
    bpy.ops.object.select_all(action='DESELECT')
    for i in range(random_view_N*2):
        bpy.ops.mesh.primitive_cone_add(radius1=0.1, radius2=0, depth=0.2, view_align=False, enter_editmode=False, location = view_infos[i][1])
        bpy.ops.transform.rotate(value = view_infos[i][2], 
                                 axis = view_infos[i][3])
        
        view_name = "view_" + str(i+1)
        bpy.context.selected_objects[0].name = view_name
        
def generatere_cfg_file(view_info):
    global camera_distance_from_object
    filename, file_extension = os.path.splitext(view_info[0])
    cfg_file = filename + ".cfg"
    
    camera_position = str(view_info[1][0]) +" "+ str(view_info[1][1]) +" "+ str(view_info[1][2]) 

    workspace_grasps = str(-camera_distance_from_object) +" "+ str(camera_distance_from_object)  +" "+ \
                       str(-camera_distance_from_object) +" "+ str(camera_distance_from_object)  +" "+ \
                       str(-camera_distance_from_object) +" "+ str(camera_distance_from_object) 

    with open(cfg_file, 'w') as f:
        f.write("workspace_grasps = %s\n" % workspace_grasps)

        f.write("camera_position = %s\n" % camera_position)
   
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
    
def test_rotation():
    versor = random_3D_unit_vector()
    print("versor: ", versor[0], versor[1], versor[2])
    distance = 1.0
    vector = [x * distance for x in versor]
    print("vector: ", vector[0], vector[1], vector[2])
    bpy.ops.mesh.primitive_cube_add(radius=0.05, location=(vector[0], vector[1], vector[2]))
    
    for i in range(30):
        vector_i = random_rotation_vector(vector, math.radians(20.0))
        bpy.ops.mesh.primitive_uv_sphere_add(size=0.05, location=(vector_i[0], vector_i[1], vector_i[2]))

    for i in range(80):
        vector_i = random_rotation_vector(vector, math.radians(45.0))
        bpy.ops.mesh.primitive_uv_sphere_add(size=0.05, location=(vector_i[0], vector_i[1], vector_i[2]))

    for i in range(120):
        vector_i = random_rotation_vector(vector, math.radians(90.0))
        bpy.ops.mesh.primitive_uv_sphere_add(size=0.05, location=(vector_i[0], vector_i[1], vector_i[2]))


reset_blend()

bpy.ops.object.lamp_add(type='POINT', view_align=False, location=(1, 1, 1))
bpy.ops.object.lamp_add(type='POINT', view_align=False, location=(1, -1, 1))
bpy.ops.object.lamp_add(type='POINT', view_align=False, location=(-1, 1, 1))
bpy.ops.object.lamp_add(type='POINT', view_align=False, location=(-1, -1, 1))
bpy.ops.object.select_all(action='DESELECT')

#Test script that generate a single view of the object
#test()
#test_rotation()

object_position = [0.0, 0.0, 0.0]
camera_distance_from_object = 0.5
angle_between_cameras = math.radians(90.0)
random_view_N = 40
pc_resolution = 300
ground_truth_leaf_size = 0.003

#X_Y_angle_limits = []#[0,np.pi/2]
#Z_angle_limits = [] #[np.pi, np.pi/2+0.4 , np.pi/2-0.4, 0]

pcd_file_prefix = "/tmp/scan_"
#obj_file = "/home/luca/CADs/soda_cans/330_can.obj"
obj_file = "/home/luca/CADs/sockets/7174K640/7174K640_SOCKET.stl"

object_name = "Mesh"
import_object(object_position, obj_file, object_name)
ground_truth_obj_file = pcd_file_prefix + "gt.obj"
export_scene(ground_truth_obj_file)

views_info = loop(camera_distance_from_object, angle_between_cameras, random_view_N, pc_resolution, pcd_file_prefix)
visualize_view_infos(views_info, random_view_N)

ground_truth_pcd_file = pcd_file_prefix + "gt.pcd"
pcl_cmd = "pcl_mesh_sampling " + ground_truth_obj_file + " " + ground_truth_pcd_file + " -leaf_size " + str(ground_truth_leaf_size)
subprocess.Popen(pcl_cmd, shell=True, stderr=subprocess.PIPE)

with open('/tmp/read_me.txt', 'w') as f:
    f.write("obj_file: %s \n" % obj_file)
    
    f.write("object_position: [")
    for item in object_position:
        f.write("%s " % item)
    f.write("]\n") 
    
    f.write("camera_distance_from_object: %s \n" % camera_distance_from_object)

    f.write("N_camera_per_view: 2\n")

    f.write("angle_between_cameras: %s \n" % angle_between_cameras)
       
    f.write("X_Y_angle_limits: [" + str(-math.pi) + " " + str(math.pi) + "]\n")
    
    f.write("Z_angle_limits: [" + str(-math.pi) + " " + str(math.pi) + "]\n")
    
    f.write("views_resolution: %s \n" % pc_resolution)

    f.write("gt_leaf_size: %s \n" % ground_truth_leaf_size)    

    f.write("random_view_N: %s \n" % random_view_N)

# to inspect the pointclouds:
# pcl_viewer scan_gt.pcd -use_point_picking

# to fuse the views:
# python two_capera_pcd_fusion.py /tmp/scan_ N