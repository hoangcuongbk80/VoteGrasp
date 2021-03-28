rosservice call /gazebo/pause_physics "{}"

rosservice call /hiqp_joint_effort_controller/remove_tasks \
"names:
- 'full_pose'
- 'ee_plane_project'
- 'approach_align'
- 'ee_cage_left'
- 'ee_cage_right'
- 'ee_cage_back'
- 'ee_cage_front'
- 'ee_obst1'
- 'ee_obst2'
- 'ee_obst3'
- 'ee_rl'
" 

rosservice call /hiqp_joint_effort_controller/remove_all_primitives "{}" 

rosservice call /hiqp_joint_effort_controller/set_tasks \
"tasks:
- name: 'full_pose'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefFullPose', '0.0', '-1.17', '0.003', '-2.89', '-0.0', '1.82', '0.84']
  dyn_params: ['TDynPD', '200.0', '29.0'] "


rosservice call /gazebo/unpause_physics "{}"

sleep 2 
