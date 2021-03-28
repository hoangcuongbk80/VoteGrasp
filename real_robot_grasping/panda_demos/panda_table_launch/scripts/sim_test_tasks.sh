rosservice call /gazebo/pause_physics "{}"
rosservice call /hiqp_joint_effort_controller/remove_tasks \
"names:
- 'full_pose'
- 'ee_to_goal'
" 
rosservice call /hiqp_joint_effort_controller/remove_all_primitives "{}" 

####################### GEOMETRIC PRIMITIVES #######################
rosservice call /hiqp_joint_effort_controller/set_primitives \
"primitives:
- name: 'ee_point'  
  type: 'point'
  frame_id: 'panda_hand'
  visible: true
  color: [0.0, 0.0, 1.0, 1.0]
  parameters: [0.0, 0.0, 0.12]
- name: 'ee_z_axis'  
  type: 'line'
  frame_id: 'panda_hand'
  visible: false
  color: [0.0, 1.0, 1.0, 1.0]
  parameters: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
- name: 'table_plane'
  type: 'plane'
  frame_id: 'world'
  visible: true
  color: [1.0, 0.0, 1.0, 1.0]   
  parameters: [0.0, 0.0, 1.0, 0.77]
- name: 'table_z_axis'  
  type: 'line'
  frame_id: 'world'
  visible: false
  color: [0.0, 1.0, 1.0, 1.0]
  parameters: [0.0, 0.0, 1.0, 0.0, -0.2, 0.7]
- name: 'goal'  
  type: 'sphere'
  frame_id: 'world'
  visible: true
  color: [0.0, 1.0, 0.0, 1.0]
  parameters: [-0.3, -0.1, 0.87, 0.02]
"

####################### TASKS #######################

rosservice call /hiqp_joint_effort_controller/set_tasks \
"tasks:
- name: 'full_pose'
  priority: 2
  visible: 1
  active: 1
  monitored: 0
  def_params: ['TDefFullPose', '0.0', '-1.17', '0.003', '-2.89', '-0.0', '1.82', '0.84']
  dyn_params: ['TDynPD', '1.0', '2.0'] 
- name: 'ee_to_goal'
  priority: 1
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefGeomProj', 'point', 'sphere', 'ee_point < goal']
  dyn_params: ['TDynPD', '1.0', '2.0']
"

#  dyn_params: ['TDynRandom', '0.0', '0.1']

#- name: 'ee_to_goal'
#  priority: 3
#  visible: 1
#  active: 0
#  monitored: 0
#  def_params: ['TDefGeomProj', 'point', 'sphere', 'ee_point < goal']
#  dyn_params: ['TDynPD', '9.0', '6.0']

#- name: 'ee_plane_project'
#  priority: 1
#  visible: 1
#  active: 1
#  monitored: 0
#  def_params: ['TDefGeomProj', 'point', 'plane', 'ee_point = table_plane']
#  dyn_params: ['TDynPD', '400.0', '51.0']
#- name: 'approach_align'
#  priority: 1
#  visible: 1
#  active: 1
#  monitored: 0
#  def_params: ['TDefGeomAlign', 'line', 'line', 'ee_z_axis = table_z_axis']
#  dyn_params: ['TDynPD', '400.0', '51.0']
#- name: 'ee_cage_front'
#  priority: 2
#  visible: 1
#  active: 1
#  monitored: 0
#  def_params: ['TDefGeomProj', 'point', 'plane', 'ee_point < front_plane']
#  dyn_params: ['TDynPD', '100.0', '21.0']
#- name: 'ee_cage_back'
#  priority: 2
#  visible: 1
#  active: 1
#  monitored: 0
#  def_params: ['TDefGeomProj', 'point', 'plane', 'ee_point > back_plane']
#  dyn_params: ['TDynPD', '100.0', '21.0']
#- name: 'ee_cage_left'
#  priority: 2
#  visible: 1
#  active: 1
#  monitored: 0
#  def_params: ['TDefGeomProj', 'point', 'plane', 'ee_point < left_plane']
#  dyn_params: ['TDynPD', '100.0', '21.0']
#- name: 'ee_cage_right'
#  priority: 2
#  visible: 1
#  active: 1
#  monitored: 0
#  def_params: ['TDefGeomProj', 'point', 'plane', 'ee_point > right_plane']
#  dyn_params: ['TDynPD', '100.0', '21.0']

rosservice call /gazebo/unpause_physics "{}"

