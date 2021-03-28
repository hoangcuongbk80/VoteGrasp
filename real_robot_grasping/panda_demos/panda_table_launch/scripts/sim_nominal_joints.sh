rosservice call /gazebo/unpause_physics "{}"
#rosservice call /hiqp_model_joint_effort_controller/remove_all_tasks "{}"

rosservice call /hiqp_joint_effort_controller/set_tasks \
"tasks:
- name: 'full_pose'
  priority: 2
  visible: 1
  active: 1
  monitored: 1
  def_params: ['TDefFullPose', '1.51', '-1.17', '0.003', '-2.89', '-0.0', '1.82', '0.84']
  dyn_params: ['TDynPD', '0.5', '1.5'] "


#def_params: ['TDefFullPose', '0.0', '-1.17', '0.003', '-2.89', '-0.0', '1.82', '0.84']
  #def_params: ['TDefFullPose', '-0.001', '-0.476', '0.003', '-2.654', '-0.0', '2.236', '0.84']
  #def_params: ['TDefFullPose', '0.007', '-0.77', '0.007', '-2.35', '0.007', '2.1', '0.9']

rosservice call /gazebo/unpause_physics "{}"
