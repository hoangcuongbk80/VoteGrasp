import bpy
import mathutils
import math
import os  

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


data_dir = '/home/hoang/OSS/VoteGrasp/blender-scripts'


with open(os.path.splitext(bpy.data.filepath)[0] + "_object_pose.txt", 'w') as fs:
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
num_view = 10
pi = math.pi
radius = 8
f = open(os.path.splitext(bpy.data.filepath)[0] + "_camera_pose.txt", "w")
f.write("image x y z rx ry rz \n")

for i in range (0, num_view):
    cam_Z = 10.0
    cam_X = radius * math.cos(i*pi/num_view)
    cam_Y = radius * math.sin(i*pi/num_view)
    cam.location = (cam_X, cam_Y, cam_Z)
    update_camera(cam)
    
    # render
    bpy.ops.render.render()
    
    old_name = os.path.join(data_dir, 'depth/Image0000.exr')
    new_name = os.path.join(data_dir, 'depth' ,str(i) + '.exr')
    os.rename(old_name, new_name)

    # save camera pose
    img_name = str(i) + '.exr'
    f.write("%s " % (img_name))
    loc = bpy.data.objects['Camera'].location
    f.write("%f %f %f " % (loc[0], loc[1], loc[2]))
    rot = bpy.data.objects['Camera'].rotation_euler
    f.write("%f %f %f\n" % (rot[0], rot[1], rot[2]))
    
f.close()