import bpy
import mathutils
import math
import os  
import shutil

data_dir = '/home/hoang/OSS/VoteGrasp/blender-scripts/data'
objects = ['011_banana', '025_mug', '024_bowl', '044_flat_screwdriver', '051_large_clamp']

if os.path.isdir(data_dir):
    shutil.rmtree(data_dir)
    
os.mkdir(data_dir)
os.mkdir(data_dir + '/depth')
os.mkdir(data_dir + '/mask')
os.mkdir(data_dir + '/semantic')
os.mkdir(data_dir + '/pointcloud')
os.mkdir(data_dir + '/obb')
os.mkdir(data_dir + '/grasp')

for obj in objects:
    os.mkdir(data_dir + '/mask/' + obj)
    
    
def update_camera(camera, focus_point=mathutils.Vector((0.0, 0.0, 0.0)), distance=1.5):
    """
    Focus the camera to a focus point and place the camera at a specific distance from that
    focus point. The camera stays in a direct line with the focus point.

    :param camera: the camera object
    :type camera: bpy.types.object
    :param focus_point: the point to focus on (default=``mathutils.Vector((0.0, 0.0, 0.0))``)
    :type focus_point: mathutils.Vector
    :param distance: the distance to keep to the focus point (default=``10.0``)
    :type distance: float
    """
    looking_direction = camera.location - focus_point
    rot_quat = looking_direction.to_track_quat('Z', 'Y')

    camera.location = (0.0, 0.0, 0.0)
    camera.rotation_euler = rot_quat.to_euler()
    camera.location = rot_quat @ mathutils.Vector((0.0, 0.0, distance))

# Save object pose
with open(data_dir + "/object_pose.txt", 'w') as fs:
    fs. write("object_ID x y z rx ry rz\n")
    for object in bpy.data.objects:
        if object.name not in ['Camera', 'Grid', 'Light']:             
            fs.write("%s " % (object.name))
            #loc = object.location * 2200
            loc = object.location
            fs.write("%f %f %f " % (loc[0], loc[1], loc[2]))
            #rot = object.rotation_quaternion
            #fs.write("%f %f %f %f\n" % (rot[1], rot[2], rot[3], rot[0]))
            rot = object.rotation_euler
            fs.write("%f %f %f\n" % (rot[0], rot[1], rot[2]))
         

cam = bpy.data.objects['Camera']
num_view = 100
pi = math.pi
radius = 8
f = open(data_dir + "/camera_pose.txt", "w")
f.write("image x y z rx ry rz \n")

for i in range (0, num_view):
    cam_Z = 10.0
    cam_X = radius * math.cos(i*pi/num_view)
    cam_Y = radius * math.sin(i*pi/num_view)
    cam.location = (cam_X, cam_Y, cam_Z)
    update_camera(cam)
    
    # render
    bpy.ops.render.render()
    
    # rename depth image and mask
    old_name = os.path.join(data_dir, 'depth/Image0000.exr')
    new_name = os.path.join(data_dir, 'depth' ,str(i) + '.exr')
    os.rename(old_name, new_name)
    for obj in objects:
        old_name = os.path.join(data_dir, 'mask/' + obj + '/Image0000.png')
        new_name = os.path.join(data_dir, 'mask/', obj, str(i) + '.png')
        os.rename(old_name, new_name)    

    # save camera pose
    img_name = str(i) + '.exr'
    f.write("%s " % (img_name))
    loc = bpy.data.objects['Camera'].location
    f.write("%f %f %f " % (loc[0], loc[1], loc[2]))
    rot = bpy.data.objects['Camera'].rotation_euler
    f.write("%f %f %f\n" % (rot[0], rot[1], rot[2]))
    
f.close()