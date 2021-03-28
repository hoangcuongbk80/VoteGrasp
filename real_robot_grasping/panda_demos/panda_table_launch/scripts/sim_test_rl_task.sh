rosservice call /hiqp_joint_effort_controller/set_tasks \
"tasks:
- name: 'full_pose'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefFullPoseRL']  
  dyn_params: ['TDynPD', '200.0', '29.0'] "

rosservice call /full_pose/act "qd: [2.007, 1.77, 0.007, -2.35, 0.007, 2.1, 0.9]"
#
rosservice call /gazebo/unpause_physics "{}"
