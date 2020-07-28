import bpy
import mathutils
import math
from time import sleep

def update_camera(camera, focus_point=mathutils.Vector((0.0, 0.0, 0.0)), distance=10.0):
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

cam = bpy.data.objects['Camera']
pi = math.pi
radius = 8

for i in range (7, 8):
    cam_Z = 5.0
    cam_X = radius * math.cos(pi/i)
    cam_Y = radius * math.sin(pi/i)
    cam.location = (cam_X, cam_Y, cam_Z)
    update_camera(cam)