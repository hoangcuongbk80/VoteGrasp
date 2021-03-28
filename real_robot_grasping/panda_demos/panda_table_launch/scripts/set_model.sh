rosservice call /gazebo/set_model_configuration \
"model_name: 'panda' 
urdf_param_name: ''
joint_names:
- 'panda_joint1'
- 'panda_joint2' 
- 'panda_joint3'
- 'panda_joint4' 
- 'panda_joint5'
- 'panda_joint6'
- 'panda_joint7'
joint_positions: [0.0, -0.8, 0.0, -2.8, 0.0, 1.8, 0.8]" 

rosservice call /gazebo/unpause_physics "{}"
