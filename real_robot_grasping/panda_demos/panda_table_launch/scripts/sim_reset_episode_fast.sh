rosservice call /hiqp_joint_effort_controller/deactivate_task "name: 'ee_rl'"
rosservice call /hiqp_joint_effort_controller/deactivate_task "name: 'ee_obst1'"
rosservice call /hiqp_joint_effort_controller/deactivate_task "name: 'ee_obst2'"
rosservice call /hiqp_joint_effort_controller/deactivate_task "name: 'ee_obst3'"
rostopic pub /ee_rl/act rl_task_plugins/DesiredErrorDynamicsMsg "e_ddot_star: [0.0, 0.0]" --once &
sleep 3
rosservice call /hiqp_joint_effort_controller/activate_task "name: 'ee_obst1'"
rosservice call /hiqp_joint_effort_controller/activate_task "name: 'ee_obst2'"
rosservice call /hiqp_joint_effort_controller/activate_task "name: 'ee_obst3'"
rosservice call /hiqp_joint_effort_controller/activate_task "name: 'ee_rl'"

