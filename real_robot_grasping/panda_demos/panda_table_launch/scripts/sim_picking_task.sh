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
  priority: 5
  visible: 1
  active: 1
  monitored: 0
  def_params: ['TDefFullPose', '0.0', '-1.17', '0.003', '-2.89', '-0.0', '1.82', '0.84']
  dyn_params: ['TDynPD', '16.0', '10.0']"

rosservice call /gazebo/unpause_physics "{}"
sleep 3

####################### GEOMETRIC PRIMITIVES #######################
rosservice call /hiqp_joint_effort_controller/set_primitives \
"primitives:
- name: 'ee_point'
  type: 'point'
  frame_id: 'panda_hand'
  visible: true
  color: [0.0, 0.0, 1.0, 1.0]
  parameters: [0.0, 0.0, 0.1]

- name: 'ee_x_axis'
  type: 'line'
  frame_id: 'panda_hand'
  visible: true
  color: [1.0, 0.0, 0.0, 1.0]
  parameters: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]

- name: 'ee_z_axis'  
  type: 'line'
  frame_id: 'panda_hand'
  visible: true
  color: [0.0, 1.0, 1.0, 1.0]
  parameters: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]

- name: 'table_plane'
  type: 'plane'
  frame_id: 'world'
  visible: true
  color: [1.0, 0.0, 1.0, 1.0]   
  parameters: [0.0, 0.0, 1.0, 0.77]


- name: 'world_x_axis'
  type: 'line'
  frame_id: 'world'
  visible: true
  color: [1.0, 0.0, 0.0, 1.0]
  parameters: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]

- name: 'table_z_axis'  
  type: 'line'
  frame_id: 'world'
  visible: true
  color: [0.0, 1.0, 1.0, 1.0]
  parameters: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]

- name: back_plane
  type: plane 
  frame_id: world
  visible: false
  color: [0.2, 0.5, 0.2, 0.21]
  parameters: [0.0, 1.0, 0.0, -0.3]

- name: front_plane
  type: plane 
  frame_id: world
  visible: false
  color: [0.2, 0.5, 0.2, 0.21]
  parameters: [0.0, 1.0, 0.0, 0.2]

- name: left_plane
  type: plane 
  frame_id: world
  visible: false
  color: [0.2, 0.5, 0.2, 0.21]
  parameters: [1.0, 0.0, 0.0, 0.1]

- name: right_plane
  type: plane 
  frame_id: world
  visible: false
  color: [0.2, 0.5, 0.2, 0.21]
  parameters: [1.0, 0.0, 0.0, -0.4]

- name: 'goal'
  type: 'box'
  frame_id: 'world'
  visible: true
  color: [0.0, 1.0, 0.0, 1.0]
  parameters: [-0.3, -0.1, 0.79, 0.04, 0.04, 0.04]

- name: 'goal_point'
  type: 'point'
  frame_id: 'world'
  visible: true
  color: [0.0, 0.0, 1.0, 1.0]
  parameters: [-0.3, -0.1, 0.79]
"


####################### TASKS #######################

rosservice call /hiqp_joint_effort_controller/set_tasks \
"tasks:
- name: 'full_pose'
  priority: 4
  visible: 1
  active: 1
  monitored: 0
  def_params: ['TDefFullPose', '0.0', '-1.17', '0.003', '-2.89', '-0.0', '1.82', '0.84']
  dyn_params: ['TDynPD', '16.0', '10.0']
- name: 'ee_point_project'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefGeomProj', 'point', 'point', 'ee_point = goal_point']
  dyn_params: ['TDynPD', '200', '500']
- name: 'approach_align_z'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefGeomAlign', 'line', 'line', 'ee_z_axis = table_z_axis']
  dyn_params: ['TDynPD', '400.0', '500.0']
- name: 'approach_align_x'
  priority: 3
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefGeomAlign', 'line', 'line', 'ee_x_axis = world_x_axis']
  dyn_params: ['TDynPD', '400.0', '500.0']
- name: 'ee_plane_project'
  priority: 1
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefGeomProj', 'point', 'plane', 'ee_point > table_plane']
  dyn_params: ['TDynPD', '400.0', '500.0']
- name: 'ee_cage_front'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefGeomProj', 'point', 'plane', 'ee_point < front_plane']
  dyn_params: ['TDynPD', '100.0', '30.0']
- name: 'ee_cage_back'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefGeomProj', 'point', 'plane', 'ee_point > back_plane']
  dyn_params: ['TDynPD', '100.0', '30.0']
- name: 'ee_cage_left'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefGeomProj', 'point', 'plane', 'ee_point < left_plane']
  dyn_params: ['TDynPD', '100.0', '30.0']
- name: 'ee_cage_right'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefGeomProj', 'point', 'plane', 'ee_point > right_plane']
  dyn_params: ['TDynPD', '100.0', '30.0']
"


