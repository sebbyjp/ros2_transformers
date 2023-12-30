Last login: Thu Dec 28 23:27:33 on ttys004
sebastianperalta@Sebastians-MacBook-Pro simply-mono % docker compose exec -it dev bash
Sourced ROS 2 humble
Sourced  dependencies workspace
^R
Sourced  base workspace
root@d3ec6893a1b6:/simply_ws# app^C
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p world_frame:=locobot_base_footprint
[INFO] [1703828595.337246919] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 13.96 seconds
[INFO] [1703828595.337295710] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703828595.486942960] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703828595.500428710] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703828595.507847502] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703828596.072177002] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.18 seconds
[INFO] [1703828596.072352377] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703828596.241122752] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703828596.408318294] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703828596.408898794] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703828596.408909002] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703828596.409197461] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703828596.409579002] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703828596.417476502] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703828596.417508252] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703828596.457128961] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703828596.457249919] [RTXAgentClientNode]: Waiting...
[INFO] [1703828596.459725627] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703828596.882156169] [RTXAgentClientNode]: Feedback callback
[INFO] [1703828596.882208586] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703828596.882224211] [RTXAgentClientNode]: Action server available after 425 ms
[ERROR] [1703828596.882326128] [RTXAgentClientNode]: move_hand began
[ERROR] [1703828596.882428711] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703828596.882488336] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703828596.882532753] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703828596.882574753] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703828596.882617169] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703828596.882345669] [RTXAgentClientNode]: VLA error: 
[INFO] [1703828596.886251378] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703828596.887466753] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703828596.893433669] [RTXAgentClientNode]: HAND OPENNESS -- Current: -1.114286, Target: 0.000000
[INFO] [1703828596.893458794] [RTXAgentClientNode]: FINGER JOINT -- Current: -0.000000, Target: 0.019500
[INFO] [1703828596.893564919] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703828596.894831419] [move_group_interface]: Planning request accepted
[INFO] [1703828596.985920711] [move_group_interface]: Planning request complete!
[INFO] [1703828596.986778836] [move_group_interface]: time taken to generate plan: 0.0296674 seconds
[INFO] [1703828596.987516253] [RTXAgentClientNode]: move_hand 104 ms
[INFO] [1703828596.989874961] [move_group_interface]: Execute request accepted
[INFO] [1703828620.723008250] [move_group_interface]: Execute request success!
[INFO] [1703828620.723064000] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703828620.723156875] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.308721, Y: 0.000327, Z: 0.247229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: -0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703828620.723170541] [RTXAgentClientNode]: Target (locobot_base_footprint frame) XYZ -- X: 0.308721, Y: 0.020327, Z: 0.227229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: 0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703828620.723284583] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703828620.726620166] [move_group_interface]: Planning request accepted
[INFO] [1703828620.883072041] [move_group_interface]: Planning request complete!
[INFO] [1703828620.883203166] [move_group_interface]: time taken to generate plan: 0.0221063 seconds
[INFO] [1703828620.883251416] [RTXAgentClientNode]: move_arm  took 160 ms
[INFO] [1703828620.884540333] [move_group_interface]: Execute request accepted
[INFO] [1703828625.844281752] [move_group_interface]: Execute request success!
[INFO] [1703828625.844982085] [RTXAgentClientNode]: Action took 29390 ms
[INFO] [1703828627.846242586] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[INFO] [1703828627.846717753] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[ros2run]: Segmentation fault
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p world_frame:=locobot_base_footprint
[INFO] [1703828780.509565379] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 84.64 seconds
[INFO] [1703828780.509762963] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703828780.659966046] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703828780.696151838] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703828780.709780546] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703828781.059774380] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.1 seconds
[INFO] [1703828781.059993921] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703828781.175404755] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703828781.419201713] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703828781.419833588] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703828781.419930088] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703828781.422007630] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703828781.425710505] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703828781.452448171] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703828781.452702671] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703828781.504454255] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703828781.504805630] [RTXAgentClientNode]: Waiting...
[INFO] [1703828781.505876588] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703828782.031007547] [RTXAgentClientNode]: Feedback callback
[INFO] [1703828782.031247297] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703828782.031326755] [RTXAgentClientNode]: Action server available after 526 ms
[ERROR] [1703828782.031347630] [RTXAgentClientNode]: move_hand began
[ERROR] [1703828782.031362380] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703828782.031372838] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703828782.031381547] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703828782.031412588] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703828782.031421672] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703828782.031572963] [RTXAgentClientNode]: VLA error: 
[INFO] [1703828782.033285755] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703828782.034398547] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703828782.095813422] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.000933, Target: 0.000000
[INFO] [1703828782.096036797] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019484, Target: 0.019500
[INFO] [1703828782.096207172] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703828782.097443338] [move_group_interface]: Planning request accepted
[INFO] [1703828782.139798338] [move_group_interface]: Planning request complete!
[INFO] [1703828782.140332088] [move_group_interface]: time taken to generate plan: 0.0132012 seconds
[INFO] [1703828782.140718338] [RTXAgentClientNode]: move_hand 109 ms
[INFO] [1703828782.143157797] [move_group_interface]: Execute request accepted
[INFO] [1703828782.330992464] [move_group_interface]: Execute request success!
[INFO] [1703828782.331241755] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703828782.331919922] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291460, Y: 0.020553, Z: 0.248874, RPY --  R: 0.002271, P: 0.899936, Y: 0.003400, Q -- W: 0.900460, X: 0.000283, Y: 0.434938, Z: 0.001037
[INFO] [1703828782.332169047] [RTXAgentClientNode]: Target (locobot_base_footprint frame) XYZ -- X: 0.291460, Y: 0.040553, Z: 0.228874, RPY --  R: 0.002271, P: 0.899936, Y: 0.003400, Q -- W: 0.900458, X: 0.001762, Y: 0.434934, Z: 0.002025
[INFO] [1703828782.332748339] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703828782.334645630] [move_group_interface]: Planning request accepted
[INFO] [1703828782.458270547] [move_group_interface]: Planning request complete!
[INFO] [1703828782.459124922] [move_group_interface]: time taken to generate plan: 0.0148028 seconds
[INFO] [1703828782.459163547] [RTXAgentClientNode]: move_arm  took 127 ms
[INFO] [1703828782.461508089] [move_group_interface]: Execute request accepted
[INFO] [1703828787.427987508] [move_group_interface]: Execute request success!
[INFO] [1703828787.428928174] [RTXAgentClientNode]: Action took 5924 ms
[INFO] [1703828789.434837800] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[INFO] [1703828789.435568550] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[ros2run]: Segmentation fault
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true
[INFO] [1703828801.684282042] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 92.44 seconds
[INFO] [1703828801.684324167] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703828801.878668542] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703828801.893491084] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703828801.897459125] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703828802.162675334] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.06 seconds
[INFO] [1703828802.162701000] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703828802.326842500] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703828802.480425667] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703828802.481614792] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703828802.481772626] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703828802.482350042] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703828802.482858251] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703828802.512324376] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703828802.512705667] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703828802.564530917] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703828802.564809876] [RTXAgentClientNode]: Waiting...
[INFO] [1703828802.566164001] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703828802.956627126] [RTXAgentClientNode]: Feedback callback
[INFO] [1703828802.956804167] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[ERROR] [1703828802.957072792] [RTXAgentClientNode]: VLA error: 
[INFO] [1703828802.961738334] [RTXAgentClientNode]: Action server available after 396 ms
[ERROR] [1703828802.961804626] [RTXAgentClientNode]: move_hand began
[ERROR] [1703828802.961820584] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703828802.961837334] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703828802.961848001] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703828802.961856584] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703828802.961919417] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703828802.965429792] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703828802.966535167] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703828802.998969001] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.000933, Target: 0.000000
[INFO] [1703828802.999356626] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019484, Target: 0.019500
[INFO] [1703828803.000047376] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703828803.001349917] [move_group_interface]: Planning request accepted
[INFO] [1703828803.036349001] [move_group_interface]: Planning request complete!
[INFO] [1703828803.036471876] [move_group_interface]: time taken to generate plan: 0.00148458 seconds
[INFO] [1703828803.036540376] [RTXAgentClientNode]: move_hand 74 ms
[INFO] [1703828803.037446542] [move_group_interface]: Execute request accepted
[INFO] [1703828803.113303917] [move_group_interface]: Execute request success!
[INFO] [1703828803.114989792] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703828803.116039126] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.274061, Y: 0.040700, Z: 0.250496, RPY --  R: 0.006886, P: 0.898981, Y: 0.007995, Q -- W: 0.900662, X: 0.001364, Y: 0.434513, Z: 0.002104
[INFO] [1703828803.116129792] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.274061, Y: 0.060700, Z: 0.230496, RPY --  R: 0.006886, P: 0.898981, Y: 0.007995, Q -- W: 0.900650, X: 0.004838, Y: 0.434488, Z: 0.005096
[INFO] [1703828803.116550709] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703828803.116923251] [move_group_interface]: Planning request accepted
[INFO] [1703828803.303214584] [move_group_interface]: Planning request complete!
[INFO] [1703828803.304158126] [move_group_interface]: time taken to generate plan: 0.0298022 seconds
[INFO] [1703828803.304183793] [RTXAgentClientNode]: move_arm  took 189 ms
[INFO] [1703828803.304536459] [move_group_interface]: Execute request accepted
[INFO] [1703828808.023190920] [move_group_interface]: Execute request success!
[INFO] [1703828808.023976711] [RTXAgentClientNode]: Action took 5459 ms
[INFO] [1703828810.025169587] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[INFO] [1703828810.025984254] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[ros2run]: Segmentation fault
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true
[INFO] [1703828817.860734716] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 98.36 seconds
[INFO] [1703828817.860795008] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703828818.032771800] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703828818.051148508] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703828818.058107341] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703828818.620006050] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.2 seconds
[INFO] [1703828818.620033258] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703828818.794829883] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703828818.909318425] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703828818.911129050] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703828818.911392217] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703828818.912417175] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703828818.913622383] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703828818.960847092] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703828818.961277217] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703828819.011529883] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703828819.013141800] [RTXAgentClientNode]: Waiting...
[INFO] [1703828819.013486217] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703828819.479239134] [RTXAgentClientNode]: Feedback callback
[INFO] [1703828819.479413550] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703828819.479471592] [RTXAgentClientNode]: Action server available after 466 ms
[ERROR] [1703828819.479491467] [RTXAgentClientNode]: move_hand began
[ERROR] [1703828819.479506967] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703828819.479517550] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703828819.479525925] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703828819.479534342] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703828819.479554925] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703828819.486062134] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703828819.493974800] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[ERROR] [1703828819.504655509] [RTXAgentClientNode]: VLA error: 
[INFO] [1703828819.555425509] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.000933, Target: 0.000000
[INFO] [1703828819.555461384] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019484, Target: 0.019500
[INFO] [1703828819.556111259] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703828819.556935425] [move_group_interface]: Planning request accepted
[INFO] [1703828819.620879884] [move_group_interface]: Planning request complete!
[INFO] [1703828819.621335342] [move_group_interface]: time taken to generate plan: 0.0242969 seconds
[INFO] [1703828819.621357759] [RTXAgentClientNode]: move_hand 137 ms
[INFO] [1703828819.621842592] [move_group_interface]: Execute request accepted
[INFO] [1703828819.678361759] [move_group_interface]: Execute request success!
[INFO] [1703828819.679069467] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703828819.679227134] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.256924, Y: 0.060419, Z: 0.252147, RPY --  R: 0.020490, P: 0.899647, Y: 0.020003, Q -- W: 0.900476, X: 0.004877, Y: 0.434854, Z: 0.004551
[INFO] [1703828819.679263259] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.256924, Y: 0.080419, Z: 0.232147, RPY --  R: 0.020490, P: 0.899647, Y: 0.020003, Q -- W: 0.900387, X: 0.013574, Y: 0.434670, Z: 0.013460
[INFO] [1703828819.679421634] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703828819.684797425] [move_group_interface]: Planning request accepted
[INFO] [1703828820.429816509] [move_group_interface]: Planning request complete!
[INFO] [1703828820.430603676] [move_group_interface]: time taken to generate plan: 0.0383763 seconds
[INFO] [1703828820.430671884] [RTXAgentClientNode]: move_arm  took 751 ms
[INFO] [1703828820.434295259] [move_group_interface]: Execute request accepted
[INFO] [1703828824.494507178] [move_group_interface]: Execute request success!
[INFO] [1703828824.494599011] [RTXAgentClientNode]: Action took 5481 ms
[INFO] [1703828826.497227929] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[INFO] [1703828826.497917012] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[ros2run]: Segmentation fault
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=3
[INFO] [1703828849.652366842] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 110 seconds
[INFO] [1703828849.652744467] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703828849.877945009] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703828849.912729300] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703828849.917636759] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703828850.358374759] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.2 seconds
[INFO] [1703828850.358399592] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703828850.497436217] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703828850.690794093] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703828850.691857176] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703828850.691882509] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703828850.692169926] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703828850.692650176] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703828850.706317843] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703828850.706384009] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703828850.750016551] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703828850.750270218] [RTXAgentClientNode]: Waiting...
[INFO] [1703828850.750802551] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703828851.388633635] [RTXAgentClientNode]: Feedback callback
[INFO] [1703828851.388885426] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[ERROR] [1703828851.389030301] [RTXAgentClientNode]: VLA error: 
[INFO] [1703828851.394068010] [RTXAgentClientNode]: Action server available after 643 ms
[ERROR] [1703828851.394158426] [RTXAgentClientNode]: move_hand began
[ERROR] [1703828851.394206010] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703828851.394244176] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703828851.394293968] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703828851.394328635] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703828851.394389760] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703828851.395430301] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703828851.396813510] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703828851.442256676] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.000933, Target: 0.000000
[INFO] [1703828851.442292718] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019484, Target: 0.019500
[INFO] [1703828851.442382760] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703828851.443080051] [move_group_interface]: Planning request accepted
[INFO] [1703828851.489070260] [move_group_interface]: Planning request complete!
[INFO] [1703828851.490206260] [move_group_interface]: time taken to generate plan: 0.013502 seconds
[INFO] [1703828851.490237176] [RTXAgentClientNode]: move_hand 95 ms
[INFO] [1703828851.491948010] [move_group_interface]: Execute request accepted
[INFO] [1703828851.544122885] [move_group_interface]: Execute request success!
[INFO] [1703828851.544650301] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703828851.544917135] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.239832, Y: 0.079415, Z: 0.253736, RPY --  R: 0.057751, P: 0.898591, Y: 0.056686, Q -- W: 0.900372, X: 0.013692, Y: 0.434713, Z: 0.012982
[INFO] [1703828851.545002968] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.239832, Y: 0.099415, Z: 0.233736, RPY --  R: 0.057751, P: 0.898591, Y: 0.056686, Q -- W: 0.899661, X: 0.038299, Y: 0.433239, Z: 0.038051
[INFO] [1703828851.545364551] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703828851.545950968] [move_group_interface]: Planning request accepted
[INFO] [1703828857.184972804] [move_group_interface]: Planning request complete!
[INFO] [1703828857.185732137] [move_group_interface]: time taken to generate plan: 5.02182 seconds
[INFO] [1703828857.185777721] [RTXAgentClientNode]: move_arm  took 5641 ms
[INFO] [1703828857.189664887] [move_group_interface]: Execute request accepted
[INFO] [1703828887.632773137] [move_group_interface]: Execute request success!
[INFO] [1703828887.633205596] [RTXAgentClientNode]: Action took 36882 ms
[INFO] [1703828889.633322472] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703828889.734115972] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703828890.107077264] [RTXAgentClientNode]: Feedback callback
[INFO] [1703828890.107215389] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[ERROR] [1703828890.107406722] [RTXAgentClientNode]: VLA error: 
[INFO] [1703828890.107491222] [RTXAgentClientNode]: Action server available after 474 ms
[ERROR] [1703828890.107556680] [RTXAgentClientNode]: move_hand began
[ERROR] [1703828890.107601055] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703828890.107646639] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703828890.107712514] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703828890.107749222] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703828890.107801347] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703828890.107851472] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.000933, Target: 0.000000
[INFO] [1703828890.107897930] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019484, Target: 0.019500
[INFO] [1703828890.108025430] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703828890.546749916] [move_group_interface]: Planning request accepted
[INFO] [1703828890.647285000] [move_group_interface]: Planning request complete!
[INFO] [1703828890.647862000] [move_group_interface]: time taken to generate plan: 0.00452675 seconds
[INFO] [1703828890.647898375] [RTXAgentClientNode]: move_hand 540 ms
[INFO] [1703828890.650481791] [move_group_interface]: Execute request accepted
[INFO] [1703828890.707548208] [move_group_interface]: Execute request success!
[INFO] [1703828890.708426916] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703828890.708502500] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.006014, Y: -0.312535, Z: 0.241553, RPY --  R: -2.650604, P: 0.257621, Y: -1.566344, Q -- W: 0.258719, X: -0.659710, Y: 0.700832, Z: -0.081745
[INFO] [1703828890.708640041] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.006014, Y: -0.292535, Z: 0.221553, RPY --  R: -2.650604, P: 0.257621, Y: -1.566344, Q -- W: 0.082896, X: -0.703762, Y: -0.656583, Z: -0.258352
[INFO] [1703828890.709050333] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703828890.710940875] [move_group_interface]: Planning request accepted
[INFO] [1703828903.436042214] [move_group_interface]: Planning request aborted
[ERROR] [1703828903.436826464] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703828903.436861381] [RTXAgentClientNode]: move_arm  took 12728 ms
[ERROR] [1703828903.436890464] [RTXAgentClientNode]: Planning failed!
[INFO] [1703828903.436926797] [RTXAgentClientNode]: Action took 13803 ms
[INFO] [1703828905.437090673] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703828905.437570423] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703828905.902361090] [RTXAgentClientNode]: Feedback callback
[INFO] [1703828905.902907590] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703828905.903730215] [RTXAgentClientNode]: Action server available after 466 ms
[ERROR] [1703828905.904030257] [RTXAgentClientNode]: move_hand began
[ERROR] [1703828905.904291590] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703828905.904660507] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703828905.905140257] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703828905.905415840] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703828905.906295715] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703828905.906448465] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.000933, Target: 0.000000
[INFO] [1703828905.906533132] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019484, Target: 0.019500
[ERROR] [1703828905.910598382] [RTXAgentClientNode]: VLA error: 
[INFO] [1703828905.914373424] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703828906.250460840] [move_group_interface]: Planning request accepted
[INFO] [1703828906.293837299] [move_group_interface]: Planning request complete!
[INFO] [1703828906.294388382] [move_group_interface]: time taken to generate plan: 0.00127013 seconds
[INFO] [1703828906.294436382] [RTXAgentClientNode]: move_hand 388 ms
[INFO] [1703828906.295063632] [move_group_interface]: Execute request accepted
[INFO] [1703828906.383257715] [move_group_interface]: Execute request success!
[INFO] [1703828906.383341049] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703828906.383402507] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.006010, Y: -0.312534, Z: 0.241553, RPY --  R: -2.650616, P: 0.257617, Y: -1.566353, Q -- W: 0.258713, X: -0.659709, Y: 0.700835, Z: -0.081743
[INFO] [1703828906.383441424] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.006010, Y: -0.292534, Z: 0.221553, RPY --  R: -2.650616, P: 0.257617, Y: -1.566353, Q -- W: 0.082892, X: -0.703759, Y: -0.656588, Z: -0.258348
[INFO] [1703828906.383544799] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703828906.384747424] [move_group_interface]: Planning request accepted
[INFO] [1703828919.177036930] [move_group_interface]: Planning request aborted
[ERROR] [1703828919.186219972] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703828919.186254722] [RTXAgentClientNode]: move_arm  took 12802 ms
[ERROR] [1703828919.186277722] [RTXAgentClientNode]: Planning failed!
[INFO] [1703828919.186284888] [RTXAgentClientNode]: Action took 13749 ms
[INFO] [1703828921.188645125] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[INFO] [1703828921.189153625] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
double free or corruption (!prev)
[ros2run]: Aborted
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true 
[INFO] [1703828936.443351840] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 142.62 seconds
[INFO] [1703828936.443439799] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703828936.612170632] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703828936.631415174] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703828936.639647424] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703828937.166079466] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.2 seconds
[INFO] [1703828937.166104341] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703828937.279355091] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703828937.526103591] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703828937.528378799] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703828937.528487966] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703828937.529848758] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703828937.531050424] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703828937.557875383] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703828937.558258008] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703828937.595287633] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703828937.596036841] [RTXAgentClientNode]: Waiting...
[INFO] [1703828937.596911883] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703828937.893072383] [RTXAgentClientNode]: Feedback callback
[INFO] [1703828937.893235633] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[ERROR] [1703828937.893423800] [RTXAgentClientNode]: VLA error: 
[INFO] [1703828937.893788716] [RTXAgentClientNode]: Action server available after 298 ms
[ERROR] [1703828937.894376300] [RTXAgentClientNode]: move_hand began
[ERROR] [1703828937.895471383] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703828937.895844050] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703828937.896040258] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703828937.896425633] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703828937.896784050] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703828937.901623133] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703828937.903080175] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703828937.919409091] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.000933, Target: 0.000000
[INFO] [1703828937.919463466] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019484, Target: 0.019500
[INFO] [1703828937.919574966] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703828937.920155341] [move_group_interface]: Planning request accepted
[INFO] [1703828937.962071883] [move_group_interface]: Planning request complete!
[INFO] [1703828937.962399675] [move_group_interface]: time taken to generate plan: 0.0122769 seconds
[INFO] [1703828937.962430466] [RTXAgentClientNode]: move_hand 65 ms
[INFO] [1703828937.962865883] [move_group_interface]: Execute request accepted
[INFO] [1703828938.075939883] [move_group_interface]: Execute request success!
[INFO] [1703828938.076525966] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703828938.076636341] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.006010, Y: -0.312534, Z: 0.241553, RPY --  R: -2.650616, P: 0.257617, Y: -1.566353, Q -- W: 0.258713, X: -0.659709, Y: 0.700835, Z: -0.081743
[INFO] [1703828938.076655466] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.006010, Y: -0.292534, Z: 0.221553, RPY --  R: -2.650616, P: 0.257617, Y: -1.566353, Q -- W: 0.082892, X: -0.703759, Y: -0.656588, Z: -0.258348
[INFO] [1703828938.076768758] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703828938.079596300] [move_group_interface]: Planning request accepted
[INFO] [1703828950.945704208] [move_group_interface]: Planning request aborted
[ERROR] [1703828950.945770875] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703828950.945968375] [RTXAgentClientNode]: move_arm  took 12869 ms
[ERROR] [1703828950.946355291] [RTXAgentClientNode]: Planning failed!
[INFO] [1703828950.946411583] [RTXAgentClientNode]: Action took 13350 ms
[INFO] [1703828952.949890959] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[INFO] [1703828952.950698667] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[ros2run]: Segmentation fault
root@d3ec6893a1b6:/simply_ws# ^C
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=3
[INFO] [1703829269.905081800] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.06 seconds
[INFO] [1703829269.905130675] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703829270.061161009] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703829270.085766842] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703829270.092417217] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703829270.443257092] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.24 seconds
[INFO] [1703829270.443287009] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703829270.540780592] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703829270.764557134] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703829270.765121884] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703829270.765175843] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703829270.765376468] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703829270.765598676] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703829270.783632843] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703829270.783722426] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703829270.831618051] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703829270.831837884] [RTXAgentClientNode]: Waiting...
[INFO] [1703829270.842912718] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829271.326295843] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829271.326387259] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703829271.326441384] [RTXAgentClientNode]: Action server available after 494 ms
[ERROR] [1703829271.326494884] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829271.326543801] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829271.326579301] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829271.326602009] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829271.326622676] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829271.326642759] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703829271.326891051] [RTXAgentClientNode]: VLA error: 
[INFO] [1703829271.327592093] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703829271.329131634] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703829271.374782843] [RTXAgentClientNode]: HAND OPENNESS -- Current: -1.114286, Target: 0.000000
[INFO] [1703829271.374820093] [RTXAgentClientNode]: FINGER JOINT -- Current: -0.000000, Target: 0.019500
[INFO] [1703829271.374980885] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829271.376669801] [move_group_interface]: Planning request accepted
[INFO] [1703829271.438769760] [move_group_interface]: Planning request complete!
[INFO] [1703829271.438965176] [move_group_interface]: time taken to generate plan: 0.0148485 seconds
[INFO] [1703829271.439048135] [RTXAgentClientNode]: move_hand 112 ms
[INFO] [1703829271.440230260] [move_group_interface]: Execute request accepted
[INFO] [1703829295.780020299] [move_group_interface]: Execute request success!
[INFO] [1703829295.780114757] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703829295.780865299] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.308721, Y: 0.000327, Z: 0.247229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: -0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703829295.780939049] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.308721, Y: 0.020327, Z: 0.227229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: 0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703829295.781605340] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829295.782903174] [move_group_interface]: Planning request accepted
[INFO] [1703829295.894801340] [move_group_interface]: Planning request complete!
[INFO] [1703829295.895025257] [move_group_interface]: time taken to generate plan: 0.0151153 seconds
[INFO] [1703829295.895059840] [RTXAgentClientNode]: move_arm  took 114 ms
[INFO] [1703829295.895914674] [move_group_interface]: Execute request accepted
[INFO] [1703829300.869287343] [move_group_interface]: Execute request success!
[INFO] [1703829300.870064426] [RTXAgentClientNode]: Action took 30038 ms
[INFO] [1703829302.870950927] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703829302.876212969] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829303.272878135] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829303.273007510] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703829303.273025594] [RTXAgentClientNode]: Action server available after 401 ms
[ERROR] [1703829303.273046760] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829303.273063010] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829303.273070302] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829303.273074719] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829303.273078510] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829303.273082719] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829303.273100427] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003502, Target: 0.000000
[INFO] [1703829303.273109177] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019439, Target: 0.019500
[INFO] [1703829303.273441635] [move_group_interface]: MoveGroup action client/server ready
[ERROR] [1703829303.273490552] [RTXAgentClientNode]: VLA error: 
[INFO] [1703829303.657993511] [move_group_interface]: Planning request accepted
[INFO] [1703829303.696246261] [move_group_interface]: Planning request complete!
[INFO] [1703829303.696968844] [move_group_interface]: time taken to generate plan: 0.0128492 seconds
[INFO] [1703829303.696994552] [RTXAgentClientNode]: move_hand 423 ms
[INFO] [1703829303.697840386] [move_group_interface]: Execute request accepted
[INFO] [1703829303.762037886] [move_group_interface]: Execute request success!
[INFO] [1703829303.763084011] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703829303.763180427] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291576, Y: 0.020328, Z: 0.248847, RPY --  R: 0.000234, P: 0.899234, Y: 0.000589, Q -- W: 0.900614, X: -0.000023, Y: 0.434621, Z: 0.000214
[INFO] [1703829303.763245261] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291576, Y: 0.040328, Z: 0.228847, RPY --  R: 0.000234, P: 0.899234, Y: 0.000589, Q -- W: 0.900614, X: 0.000233, Y: 0.434621, Z: 0.000316
[INFO] [1703829303.763387844] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829303.763924927] [move_group_interface]: Planning request accepted
[INFO] [1703829303.920411136] [move_group_interface]: Planning request complete!
[INFO] [1703829303.921071844] [move_group_interface]: time taken to generate plan: 0.0171175 seconds
[INFO] [1703829303.921120927] [RTXAgentClientNode]: move_arm  took 158 ms
[INFO] [1703829303.922735219] [move_group_interface]: Execute request accepted
[INFO] [1703829308.941678555] [move_group_interface]: Execute request success!
[INFO] [1703829308.942701055] [RTXAgentClientNode]: Action took 6071 ms
[INFO] [1703829310.942458500] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703829311.046354666] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829311.599134333] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829311.599207833] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703829311.599308500] [RTXAgentClientNode]: Action server available after 656 ms
[ERROR] [1703829311.599345750] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829311.599354708] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703829311.599496583] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829311.599513167] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829311.599522292] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829311.599530583] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829311.599538542] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829311.599619542] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003502, Target: 0.000000
[INFO] [1703829311.599638208] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019439, Target: 0.019500
[INFO] [1703829311.599798875] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829312.128634209] [move_group_interface]: Planning request accepted
[INFO] [1703829312.171773709] [move_group_interface]: Planning request complete!
[INFO] [1703829312.173053875] [move_group_interface]: time taken to generate plan: 0.0128403 seconds
[INFO] [1703829312.173087917] [RTXAgentClientNode]: move_hand 573 ms
[INFO] [1703829312.174541209] [move_group_interface]: Execute request accepted
[INFO] [1703829312.231502584] [move_group_interface]: Execute request success!
[INFO] [1703829312.234299417] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703829312.234396417] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.274424, Y: 0.040375, Z: 0.250398, RPY --  R: -0.000982, P: 0.899060, Y: -0.000315, Q -- W: 0.900651, X: -0.000374, Y: 0.434542, Z: 0.000071
[INFO] [1703829312.234423167] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.274424, Y: 0.060375, Z: 0.230398, RPY --  R: -0.000982, P: 0.899060, Y: -0.000315, Q -- W: 0.900651, X: -0.000511, Y: 0.434542, Z: -0.000355
[INFO] [1703829312.234544792] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829312.235532625] [move_group_interface]: Planning request accepted
[INFO] [1703829312.393439667] [move_group_interface]: Planning request complete!
[INFO] [1703829312.394210167] [move_group_interface]: time taken to generate plan: 0.0250831 seconds
[INFO] [1703829312.394253500] [RTXAgentClientNode]: move_arm  took 159 ms
[INFO] [1703829312.398377125] [move_group_interface]: Execute request accepted
[INFO] [1703829317.152463378] [move_group_interface]: Execute request success!
[INFO] [1703829317.153168378] [RTXAgentClientNode]: Action took 6210 ms
[INFO] [1703829319.155892545] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[INFO] [1703829319.156123420] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[ros2run]: Segmentation fault
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=3
[INFO] [1703829366.734288345] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 41.9 seconds
[INFO] [1703829366.734735929] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703829366.956279137] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703829366.987751262] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703829367.001922304] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703829367.460706596] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.14 seconds
[INFO] [1703829367.460735346] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703829367.648251512] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703829367.757443679] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703829367.758123388] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703829367.758160138] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703829367.758409888] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703829367.759361388] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703829367.809938679] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703829367.809989804] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703829367.858632846] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703829367.858969554] [RTXAgentClientNode]: Waiting...
[INFO] [1703829367.859311138] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829368.442931555] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829368.443007930] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[ERROR] [1703829368.443294555] [RTXAgentClientNode]: VLA error: 
[INFO] [1703829368.447003805] [RTXAgentClientNode]: Action server available after 588 ms
[ERROR] [1703829368.447120388] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829368.447158471] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829368.447182180] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829368.447227513] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829368.447270055] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829368.447312138] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829368.453507430] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703829368.454559846] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703829368.479158846] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003502, Target: 0.000000
[INFO] [1703829368.479247596] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019439, Target: 0.019500
[INFO] [1703829368.480277263] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829368.581489596] [move_group_interface]: Planning request accepted
[INFO] [1703829368.582130971] [move_group_interface]: Planning request complete!
[INFO] [1703829368.582361013] [move_group_interface]: time taken to generate plan: 0.0130322 seconds
[INFO] [1703829368.582410055] [RTXAgentClientNode]: move_hand 135 ms
[INFO] [1703829368.583155930] [move_group_interface]: Execute request accepted
[INFO] [1703829368.705516513] [move_group_interface]: Execute request success!
[INFO] [1703829368.706359513] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703829368.707028763] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.257224, Y: 0.060322, Z: 0.252009, RPY --  R: -0.000531, P: 0.898890, Y: -0.000460, Q -- W: 0.900688, X: -0.000139, Y: 0.434466, Z: -0.000092
[INFO] [1703829368.707536430] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.257224, Y: 0.080322, Z: 0.232009, RPY --  R: -0.000531, P: 0.898890, Y: -0.000460, Q -- W: 0.900688, X: -0.000339, Y: 0.434466, Z: -0.000322
[INFO] [1703829368.708136930] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829368.713455096] [move_group_interface]: Planning request accepted
[INFO] [1703829368.877085471] [move_group_interface]: Planning request complete!
[INFO] [1703829368.878139638] [move_group_interface]: time taken to generate plan: 0.0301997 seconds
[INFO] [1703829368.878170721] [RTXAgentClientNode]: move_arm  took 171 ms
[INFO] [1703829368.879541930] [move_group_interface]: Execute request accepted
[INFO] [1703829374.086563835] [move_group_interface]: Execute request success!
[INFO] [1703829374.087295626] [RTXAgentClientNode]: Action took 6228 ms
[INFO] [1703829376.087439294] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703829376.088086961] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829376.724498378] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829376.724739253] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703829376.724985878] [RTXAgentClientNode]: Action server available after 637 ms
[ERROR] [1703829376.725241336] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829376.725314419] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829376.725366836] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829376.725425544] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829376.725488211] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829376.725554378] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829376.725631836] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003502, Target: 0.000000
[INFO] [1703829376.725711419] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019439, Target: 0.019500
[INFO] [1703829376.725910669] [move_group_interface]: MoveGroup action client/server ready
[ERROR] [1703829376.726401419] [RTXAgentClientNode]: VLA error: 
[INFO] [1703829377.029848086] [move_group_interface]: Planning request accepted
[INFO] [1703829377.104150878] [move_group_interface]: Planning request complete!
[INFO] [1703829377.104386128] [move_group_interface]: time taken to generate plan: 0.00387154 seconds
[INFO] [1703829377.105800544] [RTXAgentClientNode]: move_hand 380 ms
[INFO] [1703829377.112757211] [move_group_interface]: Execute request accepted
[INFO] [1703829377.178149503] [move_group_interface]: Execute request success!
[INFO] [1703829377.182697086] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703829377.182749336] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.240064, Y: 0.080288, Z: 0.253593, RPY --  R: -0.001214, P: 0.899860, Y: -0.000081, Q -- W: 0.900477, X: -0.000529, Y: 0.434902, Z: 0.000227
[INFO] [1703829377.182763128] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.240064, Y: 0.100288, Z: 0.233593, RPY --  R: -0.001214, P: 0.899860, Y: -0.000081, Q -- W: 0.900477, X: -0.000564, Y: 0.434902, Z: -0.000300
[INFO] [1703829377.182896753] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829377.183640628] [move_group_interface]: Planning request accepted
[INFO] [1703829377.286733794] [move_group_interface]: Planning request complete!
[INFO] [1703829377.287413253] [move_group_interface]: time taken to generate plan: 0.0151573 seconds
[INFO] [1703829377.287471544] [RTXAgentClientNode]: move_arm  took 104 ms
[INFO] [1703829377.288184544] [move_group_interface]: Execute request accepted
[INFO] [1703829381.820707297] [move_group_interface]: Execute request success!
[INFO] [1703829381.820733255] [RTXAgentClientNode]: Action took 5733 ms
[INFO] [1703829383.820912589] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703829383.821901589] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829384.195454214] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829384.195595464] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703829384.195619339] [RTXAgentClientNode]: Action server available after 374 ms
[ERROR] [1703829384.197429298] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829384.197520214] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829384.197795756] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829384.198003964] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829384.195645839] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703829384.198148964] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829384.198177214] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829384.198210923] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003502, Target: 0.000000
[INFO] [1703829384.198424839] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019439, Target: 0.019500
[INFO] [1703829384.198878381] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829384.605854215] [move_group_interface]: Planning request accepted
[INFO] [1703829384.664067215] [move_group_interface]: Planning request complete!
[INFO] [1703829384.664303548] [move_group_interface]: time taken to generate plan: 0.0148777 seconds
[INFO] [1703829384.664333090] [RTXAgentClientNode]: move_hand 466 ms
[INFO] [1703829384.669366923] [move_group_interface]: Execute request accepted
[INFO] [1703829384.826595340] [move_group_interface]: Execute request success!
[INFO] [1703829384.826645048] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703829384.826731881] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.222999, Y: 0.100378, Z: 0.255213, RPY --  R: -0.001917, P: 0.900813, Y: -0.000756, Q -- W: 0.900270, X: -0.000698, Y: 0.435332, Z: 0.000077
[INFO] [1703829384.826750506] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.222999, Y: 0.120378, Z: 0.235213, RPY --  R: -0.001917, P: 0.900813, Y: -0.000756, Q -- W: 0.900270, X: -0.001027, Y: 0.435331, Z: -0.000757
[INFO] [1703829384.826885756] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829384.829050965] [move_group_interface]: Planning request accepted
[INFO] [1703829385.061274173] [move_group_interface]: Planning request complete!
[INFO] [1703829385.061658715] [move_group_interface]: time taken to generate plan: 0.0244335 seconds
[INFO] [1703829385.061747090] [RTXAgentClientNode]: move_arm  took 235 ms
[INFO] [1703829385.062201048] [move_group_interface]: Execute request accepted
[INFO] [1703829389.230792258] [move_group_interface]: Execute request success!
[INFO] [1703829389.230995342] [RTXAgentClientNode]: Action took 5410 ms
[INFO] [1703829391.233390509] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[INFO] [1703829391.233815551] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[ros2run]: Segmentation fault
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=3
[INFO] [1703829412.975793922] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 58.72 seconds
[INFO] [1703829412.975840880] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703829413.176066297] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703829413.199600214] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703829413.210792172] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703829413.685013048] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.2 seconds
[INFO] [1703829413.685061339] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703829413.820687048] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703829414.011651506] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703829414.012221673] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703829414.012237381] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703829414.012570214] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703829414.012801298] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703829414.027905881] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703829414.027968339] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703829414.064347464] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703829414.064595214] [RTXAgentClientNode]: Waiting...
[INFO] [1703829414.065003131] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829414.472495215] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829414.472694923] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703829414.472873798] [RTXAgentClientNode]: Action server available after 408 ms
[ERROR] [1703829414.472914673] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703829414.473204215] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829414.473217340] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829414.473227340] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829414.473235423] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829414.473417881] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829414.473430006] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829414.474936923] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703829414.476445006] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703829414.480565298] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003502, Target: 0.000000
[INFO] [1703829414.480582798] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019439, Target: 0.019500
[INFO] [1703829414.480669215] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829414.763826548] [move_group_interface]: Planning request accepted
[INFO] [1703829414.859439340] [move_group_interface]: Planning request complete!
[INFO] [1703829414.859506423] [move_group_interface]: time taken to generate plan: 0.0186559 seconds
[INFO] [1703829414.859553590] [RTXAgentClientNode]: move_hand 386 ms
[INFO] [1703829414.860867006] [move_group_interface]: Execute request accepted
[INFO] [1703829414.914663256] [move_group_interface]: Execute request success!
[INFO] [1703829414.915712173] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703829414.915818256] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.205860, Y: 0.120341, Z: 0.256849, RPY --  R: -0.004047, P: 0.901705, Y: -0.002740, Q -- W: 0.900075, X: -0.001224, Y: 0.435734, Z: -0.000352
[INFO] [1703829414.915831715] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.205860, Y: 0.140341, Z: 0.236849, RPY --  R: -0.004047, P: 0.901705, Y: -0.002740, Q -- W: 0.900072, X: -0.002418, Y: 0.435729, Z: -0.002115
[INFO] [1703829414.915938006] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829414.916212423] [move_group_interface]: Planning request accepted
[INFO] [1703829415.088744048] [move_group_interface]: Planning request complete!
[INFO] [1703829415.089650882] [move_group_interface]: time taken to generate plan: 0.014248 seconds
[INFO] [1703829415.090607298] [RTXAgentClientNode]: move_arm  took 174 ms
[INFO] [1703829415.191619590] [move_group_interface]: Execute request accepted
[INFO] [1703829418.704118717] [move_group_interface]: Execute request success!
[INFO] [1703829418.704722967] [RTXAgentClientNode]: Action took 4640 ms
[INFO] [1703829420.707259176] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703829420.708193468] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829421.359769884] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829421.360098509] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[ERROR] [1703829421.360342634] [RTXAgentClientNode]: VLA error: 
[INFO] [1703829421.361309176] [RTXAgentClientNode]: Action server available after 653 ms
[ERROR] [1703829421.361407968] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829421.361423134] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829421.361435343] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829421.361443509] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829421.361451926] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829421.361460426] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829421.361507718] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003502, Target: 0.000000
[INFO] [1703829421.361535176] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019439, Target: 0.019500
[INFO] [1703829421.361691593] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829421.769401801] [move_group_interface]: Planning request accepted
[INFO] [1703829421.769823551] [move_group_interface]: Planning request complete!
[INFO] [1703829421.770510260] [move_group_interface]: time taken to generate plan: 0.0139673 seconds
[INFO] [1703829421.770536051] [RTXAgentClientNode]: move_hand 409 ms
[INFO] [1703829421.770871593] [move_group_interface]: Execute request accepted
[INFO] [1703829421.827211385] [move_group_interface]: Execute request success!
[INFO] [1703829421.827546093] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703829421.827606718] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.188807, Y: 0.140473, Z: 0.258480, RPY --  R: -0.011214, P: 0.902463, Y: -0.010467, Q -- W: 0.899897, X: -0.002764, Y: 0.436088, Z: -0.002264
[INFO] [1703829421.827622676] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.188807, Y: 0.160473, Z: 0.238480, RPY --  R: -0.011214, P: 0.902463, Y: -0.010467, Q -- W: 0.899872, X: -0.007328, Y: 0.436035, Z: -0.007155
[INFO] [1703829421.827726093] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829421.828399760] [move_group_interface]: Planning request accepted
[INFO] [1703829422.049497343] [move_group_interface]: Planning request complete!
[INFO] [1703829422.052269801] [move_group_interface]: time taken to generate plan: 0.0223415 seconds
[INFO] [1703829422.052290301] [RTXAgentClientNode]: move_arm  took 224 ms
[INFO] [1703829422.157240843] [move_group_interface]: Execute request accepted
[INFO] [1703829425.121148720] [move_group_interface]: Execute request success!
[INFO] [1703829425.122368428] [RTXAgentClientNode]: Action took 4414 ms
[INFO] [1703829427.122571262] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703829427.123033721] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829427.662081179] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829427.662263888] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[ERROR] [1703829427.662322554] [RTXAgentClientNode]: VLA error: 
[INFO] [1703829427.696899096] [RTXAgentClientNode]: Action server available after 574 ms
[ERROR] [1703829427.696943596] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829427.696949388] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829427.696958263] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829427.696962304] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829427.696966554] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829427.696971054] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829427.696992554] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003502, Target: 0.000000
[INFO] [1703829427.697004554] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019439, Target: 0.019500
[INFO] [1703829427.697109179] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829428.073974221] [move_group_interface]: Planning request accepted
[INFO] [1703829428.236185346] [move_group_interface]: Planning request complete!
[INFO] [1703829428.236311638] [move_group_interface]: time taken to generate plan: 0.0150843 seconds
[INFO] [1703829428.236394513] [RTXAgentClientNode]: move_hand 539 ms
[INFO] [1703829428.237226263] [move_group_interface]: Execute request accepted
[INFO] [1703829428.346817054] [move_group_interface]: Execute request success!
[INFO] [1703829428.347556096] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703829428.347620054] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.171773, Y: 0.160956, Z: 0.260082, RPY --  R: -0.032280, P: 0.902552, Y: -0.032496, Q -- W: 0.899770, X: -0.007437, Y: 0.436236, Z: -0.007581
[INFO] [1703829428.347647971] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.171773, Y: 0.180956, Z: 0.240082, RPY --  R: -0.032280, P: 0.902552, Y: -0.032496, Q -- W: 0.899541, X: -0.021606, Y: 0.435764, Z: -0.021656
[INFO] [1703829428.347741804] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829428.348029554] [move_group_interface]: Planning request accepted
[INFO] [1703829428.604660388] [move_group_interface]: Planning request complete!
[INFO] [1703829428.605052305] [move_group_interface]: time taken to generate plan: 0.0210522 seconds
[INFO] [1703829428.605113138] [RTXAgentClientNode]: move_arm  took 257 ms
[INFO] [1703829428.605811013] [move_group_interface]: Execute request accepted
[INFO] [1703829431.545748750] [move_group_interface]: Execute request success!
[INFO] [1703829431.546146250] [RTXAgentClientNode]: Action took 4423 ms
[INFO] [1703829433.547451793] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[INFO] [1703829433.547801334] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[ros2run]: Segmentation fault
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=3
[INFO] [1703829508.097020925] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 95.04 seconds
[INFO] [1703829508.097151800] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703829508.287802633] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703829508.301826091] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703829508.309069550] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703829508.858870508] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.22 seconds
[INFO] [1703829508.858912925] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703829509.122532842] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703829509.366946175] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703829509.369110092] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703829509.369278259] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703829509.369983050] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703829509.370583217] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703829509.402856050] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703829509.402924009] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703829509.456799759] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703829509.457075217] [RTXAgentClientNode]: Waiting...
[INFO] [1703829519.693558722] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829520.527996625] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829520.528192333] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[ERROR] [1703829520.528313625] [RTXAgentClientNode]: VLA error: 
[INFO] [1703829520.529096833] [RTXAgentClientNode]: Action server available after 11073 ms
[ERROR] [1703829520.529307625] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829520.529381833] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829520.529544458] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829520.529597708] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829520.529669750] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829520.529719333] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829520.531754416] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703829520.533009291] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703829520.552887541] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003502, Target: 0.000000
[INFO] [1703829520.553976750] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019439, Target: 0.019500
[INFO] [1703829520.555958625] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829520.557496583] [move_group_interface]: Planning request accepted
[INFO] [1703829520.684273750] [move_group_interface]: Planning request complete!
[INFO] [1703829520.684938666] [move_group_interface]: time taken to generate plan: 0.0306179 seconds
[INFO] [1703829520.685008958] [RTXAgentClientNode]: move_hand 155 ms
[INFO] [1703829520.686401666] [move_group_interface]: Execute request accepted
[INFO] [1703829520.751150208] [move_group_interface]: Execute request success!
[INFO] [1703829520.751972125] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703829520.752085416] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.154727, Y: 0.182633, Z: 0.261668, RPY --  R: -0.093672, P: 0.899822, Y: -0.092987, Q -- W: 0.899473, X: -0.021924, Y: 0.435899, Z: -0.021467
[INFO] [1703829520.752134208] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.154727, Y: 0.162633, Z: 0.281668, RPY --  R: -0.093672, P: 0.899822, Y: -0.092987, Q -- W: 0.897580, X: -0.062304, Y: 0.431980, Z: -0.062145
[INFO] [1703829520.753108375] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829520.759421708] [move_group_interface]: Planning request accepted
[INFO] [1703829520.914253500] [move_group_interface]: Planning request complete!
[INFO] [1703829520.914360750] [move_group_interface]: time taken to generate plan: 0.0163758 seconds
[INFO] [1703829520.914494833] [RTXAgentClientNode]: move_arm  took 162 ms
[INFO] [1703829520.915070208] [move_group_interface]: Execute request accepted
[INFO] [1703829525.449606044] [move_group_interface]: Execute request success!
[INFO] [1703829525.449960627] [RTXAgentClientNode]: Action took 15994 ms
[INFO] [1703829527.450074836] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703829527.456370586] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829528.159216337] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829528.159351295] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703829528.159403670] [RTXAgentClientNode]: Action server available after 709 ms
[ERROR] [1703829528.159517170] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829528.159585462] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829528.159647170] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829528.159766545] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829528.159932920] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829528.160021587] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829528.160126670] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003502, Target: 0.000000
[INFO] [1703829528.160185795] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019439, Target: 0.019500
[INFO] [1703829528.160520378] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829528.161021795] [move_group_interface]: Planning request accepted
[ERROR] [1703829528.161119420] [RTXAgentClientNode]: VLA error: 
[INFO] [1703829528.254001295] [move_group_interface]: Planning request complete!
[INFO] [1703829528.254682295] [move_group_interface]: time taken to generate plan: 0.0155935 seconds
[INFO] [1703829528.254758920] [RTXAgentClientNode]: move_hand 94 ms
[INFO] [1703829528.255509670] [move_group_interface]: Execute request accepted
[INFO] [1703829528.308965628] [move_group_interface]: Execute request success!
[INFO] [1703829528.309510170] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703829528.309567295] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.137734, Y: 0.167178, Z: 0.302807, RPY --  R: -0.261937, P: 0.875986, Y: -0.262488, Q -- W: 0.897376, X: -0.062221, Y: 0.432352, Z: -0.062587
[INFO] [1703829528.309588628] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.137734, Y: 0.147178, Z: 0.322807, RPY --  R: -0.261937, P: 0.875986, Y: -0.262488, Q -- W: 0.882879, X: -0.172278, Y: 0.401397, Z: -0.172411
[INFO] [1703829528.309733087] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829528.310939462] [move_group_interface]: Planning request accepted
[INFO] [1703829528.574557545] [move_group_interface]: Planning request complete!
[INFO] [1703829528.574932212] [move_group_interface]: time taken to generate plan: 0.0147467 seconds
[INFO] [1703829528.575276462] [RTXAgentClientNode]: move_arm  took 265 ms
[INFO] [1703829528.576311253] [move_group_interface]: Execute request accepted
[INFO] [1703829534.044058506] [move_group_interface]: Execute request success!
[INFO] [1703829534.045026214] [RTXAgentClientNode]: Action took 6594 ms
[INFO] [1703829536.046192924] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703829536.050504632] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829536.576649466] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829536.577092466] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703829536.577349091] [RTXAgentClientNode]: Action server available after 531 ms
[ERROR] [1703829536.577383632] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829536.577391091] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829536.577398424] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829536.577402632] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829536.577406591] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829536.577412007] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829536.577429257] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003502, Target: 0.000000
[INFO] [1703829536.577434591] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019439, Target: 0.019500
[INFO] [1703829536.577580049] [move_group_interface]: MoveGroup action client/server ready
[ERROR] [1703829536.577949882] [RTXAgentClientNode]: VLA error: 
[INFO] [1703829536.578465424] [move_group_interface]: Planning request accepted
[INFO] [1703829536.647118132] [move_group_interface]: Planning request complete!
[INFO] [1703829536.647549674] [move_group_interface]: time taken to generate plan: 0.016653 seconds
[INFO] [1703829536.647608174] [RTXAgentClientNode]: move_hand 70 ms
[INFO] [1703829536.648579007] [move_group_interface]: Execute request accepted
[INFO] [1703829536.793423382] [move_group_interface]: Execute request success!
[INFO] [1703829536.793620049] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703829536.793679007] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.120682, Y: 0.159431, Z: 0.340775, RPY --  R: -0.620920, P: 0.706884, Y: -0.620306, Q -- W: 0.882984, X: -0.172348, Y: 0.401332, Z: -0.171954
[INFO] [1703829536.793727466] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.120682, Y: 0.139431, Z: 0.360775, RPY --  R: -0.620920, P: 0.706884, Y: -0.620306, Q -- W: 0.818438, X: -0.373528, Y: 0.226381, Z: -0.373346
[INFO] [1703829536.793867674] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829536.794951674] [move_group_interface]: Planning request accepted
[INFO] [1703829536.931058716] [move_group_interface]: Planning request complete!
[INFO] [1703829536.931142257] [move_group_interface]: time taken to generate plan: 0.0270133 seconds
[INFO] [1703829536.931210841] [RTXAgentClientNode]: move_arm  took 137 ms
[INFO] [1703829536.932249424] [move_group_interface]: Execute request accepted
[INFO] [1703829543.097521969] [move_group_interface]: Execute request success!
[INFO] [1703829543.097927177] [RTXAgentClientNode]: Action took 7051 ms
[INFO] [1703829545.101476011] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[INFO] [1703829545.102701511] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
double free or corruption (!prev)
[ros2run]: Aborted
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=3
[INFO] [1703829606.690607512] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 1.92 seconds
[INFO] [1703829606.690662345] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703829606.920878429] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703829606.943215720] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703829606.952687470] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703829607.387456846] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.06 seconds
[INFO] [1703829607.387614387] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703829607.568448096] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703829607.765620429] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703829607.766379971] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703829607.766394471] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703829607.766642638] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703829607.766901304] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703829607.792680096] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703829607.792865763] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703829607.832543721] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703829607.832886388] [RTXAgentClientNode]: Waiting...
[INFO] [1703829607.833564971] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829608.297671304] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829608.297991346] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703829608.298065679] [RTXAgentClientNode]: Action server available after 465 ms
[ERROR] [1703829608.298086679] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829608.298136304] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829608.298155179] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829608.298171888] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829608.298187513] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829608.298211054] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703829608.298302846] [RTXAgentClientNode]: VLA error: 
[INFO] [1703829608.302350846] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703829608.304301138] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703829608.311945679] [RTXAgentClientNode]: HAND OPENNESS -- Current: -1.114286, Target: 0.000000
[INFO] [1703829608.312021471] [RTXAgentClientNode]: FINGER JOINT -- Current: -0.000000, Target: 0.019500
[INFO] [1703829608.312491179] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829608.313045929] [move_group_interface]: Planning request accepted
[INFO] [1703829608.361569596] [move_group_interface]: Planning request complete!
[INFO] [1703829608.362370888] [move_group_interface]: time taken to generate plan: 0.0164656 seconds
[INFO] [1703829608.362650679] [RTXAgentClientNode]: move_hand 64 ms
[INFO] [1703829608.365835763] [move_group_interface]: Execute request accepted
[INFO] [1703829633.962026802] [move_group_interface]: Execute request success!
[INFO] [1703829633.962119344] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703829633.962236052] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.308721, Y: 0.000327, Z: 0.247229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: -0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703829633.962264844] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.308721, Y: -0.019673, Z: 0.267229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: 0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703829633.962376886] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829633.965404594] [move_group_interface]: Planning request accepted
[INFO] [1703829634.147043552] [move_group_interface]: Planning request complete!
[INFO] [1703829634.147079511] [move_group_interface]: time taken to generate plan: 0.0218709 seconds
[INFO] [1703829634.147485094] [RTXAgentClientNode]: move_arm  took 185 ms
[INFO] [1703829634.149105594] [move_group_interface]: Execute request accepted
[INFO] [1703829639.316529263] [move_group_interface]: Execute request success!
[INFO] [1703829639.317405763] [RTXAgentClientNode]: Action took 31484 ms
[INFO] [1703829641.317147083] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703829641.317656792] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829642.007795292] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829642.008106084] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703829642.008226000] [RTXAgentClientNode]: Action server available after 691 ms
[ERROR] [1703829642.008373959] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829642.008435000] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829642.008519875] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829642.008559250] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829642.008613500] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829642.008650667] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703829642.008408125] [RTXAgentClientNode]: VLA error: 
[INFO] [1703829642.008745334] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003857, Target: 0.000000
[INFO] [1703829642.008795584] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019433, Target: 0.019500
[INFO] [1703829642.008927459] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829642.533760709] [move_group_interface]: Planning request accepted
[INFO] [1703829642.631573501] [move_group_interface]: Planning request complete!
[INFO] [1703829642.632527084] [move_group_interface]: time taken to generate plan: 0.0232401 seconds
[INFO] [1703829642.632554709] [RTXAgentClientNode]: move_hand 623 ms
[INFO] [1703829642.632976084] [move_group_interface]: Execute request accepted
[INFO] [1703829642.702427709] [move_group_interface]: Execute request success!
[INFO] [1703829642.702903084] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703829642.703518251] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291616, Y: -0.019668, Z: 0.288846, RPY --  R: 0.000511, P: 0.900281, Y: 0.000148, Q -- W: 0.900386, X: 0.000198, Y: 0.435092, Z: -0.000044
[INFO] [1703829642.703690001] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291616, Y: -0.039668, Z: 0.308846, RPY --  R: 0.000511, P: 0.900281, Y: 0.000148, Q -- W: 0.900386, X: 0.000262, Y: 0.435092, Z: 0.000178
[INFO] [1703829642.704126042] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829642.704973626] [move_group_interface]: Planning request accepted
[INFO] [1703829642.868805959] [move_group_interface]: Planning request complete!
[INFO] [1703829642.869785251] [move_group_interface]: time taken to generate plan: 0.0166634 seconds
[INFO] [1703829642.869813334] [RTXAgentClientNode]: move_arm  took 166 ms
[INFO] [1703829642.870224042] [move_group_interface]: Execute request accepted
[INFO] [1703829647.642389503] [move_group_interface]: Execute request success!
[INFO] [1703829647.642920878] [RTXAgentClientNode]: Action took 6325 ms
[INFO] [1703829649.643069212] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703829649.743798046] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829650.017126087] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829650.017210296] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[ERROR] [1703829650.017331796] [RTXAgentClientNode]: VLA error: 
[INFO] [1703829650.017864337] [RTXAgentClientNode]: Action server available after 374 ms
[ERROR] [1703829650.017914962] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829650.017930879] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829650.017944754] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829650.017954046] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829650.017963129] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829650.017972587] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829650.018008462] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003857, Target: 0.000000
[INFO] [1703829650.018043879] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019433, Target: 0.019500
[INFO] [1703829650.018999796] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829650.302615463] [move_group_interface]: Planning request accepted
[INFO] [1703829650.356056296] [move_group_interface]: Planning request complete!
[INFO] [1703829650.357095296] [move_group_interface]: time taken to generate plan: 0.0121957 seconds
[INFO] [1703829650.357223338] [RTXAgentClientNode]: move_hand 339 ms
[INFO] [1703829650.361529421] [move_group_interface]: Execute request accepted
[INFO] [1703829650.465031921] [move_group_interface]: Execute request success!
[INFO] [1703829650.466574379] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703829650.466758296] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.274422, Y: -0.039711, Z: 0.330391, RPY --  R: 0.001178, P: 0.900209, Y: 0.002181, Q -- W: 0.900401, X: 0.000056, Y: 0.435060, Z: 0.000726
[INFO] [1703829650.466833213] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.274422, Y: -0.059711, Z: 0.350391, RPY --  R: 0.001178, P: 0.900209, Y: 0.002181, Q -- W: 0.900401, X: 0.001005, Y: 0.435059, Z: 0.001238
[INFO] [1703829650.466994213] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829650.467326379] [move_group_interface]: Planning request accepted
[INFO] [1703829650.588863338] [move_group_interface]: Planning request complete!
[INFO] [1703829650.589308546] [move_group_interface]: time taken to generate plan: 0.0260738 seconds
[INFO] [1703829650.589676046] [RTXAgentClientNode]: move_arm  took 123 ms
[INFO] [1703829650.590651213] [move_group_interface]: Execute request accepted
[INFO] [1703829654.697409298] [move_group_interface]: Execute request success!
[INFO] [1703829654.697542006] [RTXAgentClientNode]: Action took 5054 ms
[INFO] [1703829656.702877674] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[INFO] [1703829656.703808716] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[ros2run]: Segmentation fault
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=10
[INFO] [1703829667.090160971] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 23.82 seconds
[INFO] [1703829667.094978304] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703829667.272635429] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703829667.288547596] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703829667.331209762] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703829667.628816804] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.1 seconds
[INFO] [1703829667.628903304] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703829667.866172638] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703829668.026310388] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703829668.027032763] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703829668.027047263] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703829668.027348429] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703829668.027675721] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703829668.047558346] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703829668.047589304] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703829668.125010721] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703829668.125126388] [RTXAgentClientNode]: Waiting...
[INFO] [1703829668.126301179] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829668.548941096] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829668.549008680] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[ERROR] [1703829668.549142471] [RTXAgentClientNode]: VLA error: 
[INFO] [1703829668.551719596] [RTXAgentClientNode]: Action server available after 426 ms
[ERROR] [1703829668.551755680] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829668.551768763] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829668.551792013] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829668.551802263] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829668.551810305] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829668.551820263] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829668.553024138] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703829668.555335180] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703829668.615567388] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003857, Target: 0.000000
[INFO] [1703829668.615736721] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019433, Target: 0.019500
[INFO] [1703829668.615939346] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829668.885123013] [move_group_interface]: Planning request accepted
[INFO] [1703829669.032009430] [move_group_interface]: Planning request complete!
[INFO] [1703829669.032314388] [move_group_interface]: time taken to generate plan: 0.0127588 seconds
[INFO] [1703829669.032407471] [RTXAgentClientNode]: move_hand 480 ms
[INFO] [1703829669.033607763] [move_group_interface]: Execute request accepted
[INFO] [1703829669.140225847] [move_group_interface]: Execute request success!
[INFO] [1703829669.140759013] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703829669.140949555] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.257343, Y: -0.059810, Z: 0.372029, RPY --  R: 0.005493, P: 0.900957, Y: 0.005950, Q -- W: 0.900235, X: 0.001177, Y: 0.435400, Z: 0.001482
[INFO] [1703829669.141068972] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.257343, Y: -0.079810, Z: 0.392029, RPY --  R: 0.005493, P: 0.900957, Y: 0.005950, Q -- W: 0.900228, X: 0.003768, Y: 0.435385, Z: 0.003874
[INFO] [1703829669.141359097] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829669.142194388] [move_group_interface]: Planning request accepted
[INFO] [1703829669.318342597] [move_group_interface]: Planning request complete!
[INFO] [1703829669.320472472] [move_group_interface]: time taken to generate plan: 0.0168897 seconds
[INFO] [1703829669.320496055] [RTXAgentClientNode]: move_arm  took 179 ms
[INFO] [1703829669.320867805] [move_group_interface]: Execute request accepted
[INFO] [1703829673.753560793] [move_group_interface]: Execute request success!
[INFO] [1703829673.755892543] [RTXAgentClientNode]: Action took 5624 ms
[INFO] [1703829675.756047835] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703829675.756996544] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829676.138650127] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829676.138781586] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703829676.138836336] [RTXAgentClientNode]: Action server available after 382 ms
[ERROR] [1703829676.139012544] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829676.139039752] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703829676.139184711] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829676.139257294] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829676.139308502] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829676.139354544] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829676.139409461] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829676.139483086] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003857, Target: 0.000000
[INFO] [1703829676.139526127] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019433, Target: 0.019500
[INFO] [1703829676.139686169] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829676.579506169] [move_group_interface]: Planning request accepted
[INFO] [1703829676.657081336] [move_group_interface]: Planning request complete!
[INFO] [1703829676.657103169] [move_group_interface]: time taken to generate plan: 0.013284 seconds
[INFO] [1703829676.657447669] [RTXAgentClientNode]: move_hand 517 ms
[INFO] [1703829676.659368336] [move_group_interface]: Execute request accepted
[INFO] [1703829676.731570336] [move_group_interface]: Execute request success!
[INFO] [1703829676.732554919] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703829676.732614628] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.240323, Y: -0.080078, Z: 0.413618, RPY --  R: 0.015128, P: 0.901461, Y: 0.015753, Q -- W: 0.900101, X: 0.003377, Y: 0.435651, Z: 0.003795
[INFO] [1703829676.732633419] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.240323, Y: -0.100078, Z: 0.433618, RPY --  R: 0.015128, P: 0.901461, Y: 0.015753, Q -- W: 0.900049, X: 0.010239, Y: 0.435544, Z: 0.010384
[INFO] [1703829676.732745003] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829676.737497669] [move_group_interface]: Planning request accepted
[INFO] [1703829677.085727044] [move_group_interface]: Planning request aborted
[ERROR] [1703829677.086566211] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703829677.086603253] [RTXAgentClientNode]: move_arm  took 354 ms
[ERROR] [1703829677.086622753] [RTXAgentClientNode]: Planning failed!
[INFO] [1703829677.086633628] [RTXAgentClientNode]: Action took 1330 ms
[INFO] [1703829679.086761087] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703829679.090609962] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829679.559744879] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829679.559881462] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703829679.560116296] [RTXAgentClientNode]: Action server available after 473 ms
[ERROR] [1703829679.560220254] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829679.560279629] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829679.560328879] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829679.560375421] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829679.560413879] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829679.560497587] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829679.560556212] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003857, Target: 0.000000
[INFO] [1703829679.560597921] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019433, Target: 0.019500
[ERROR] [1703829679.560711712] [RTXAgentClientNode]: VLA error: 
[INFO] [1703829679.560829879] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829679.602859796] [move_group_interface]: Planning request accepted
[INFO] [1703829679.676223796] [move_group_interface]: Planning request complete!
[INFO] [1703829679.678546046] [move_group_interface]: time taken to generate plan: 0.0137335 seconds
[INFO] [1703829679.678593337] [RTXAgentClientNode]: move_hand 118 ms
[INFO] [1703829679.679845129] [move_group_interface]: Execute request accepted
[INFO] [1703829679.736306421] [move_group_interface]: Execute request success!
[INFO] [1703829679.737270712] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703829679.737331879] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.240321, Y: -0.080080, Z: 0.413621, RPY --  R: 0.015127, P: 0.901461, Y: 0.015754, Q -- W: 0.900102, X: 0.003377, Y: 0.435651, Z: 0.003795
[INFO] [1703829679.737348712] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.240321, Y: -0.100080, Z: 0.433621, RPY --  R: 0.015127, P: 0.901461, Y: 0.015754, Q -- W: 0.900050, X: 0.010239, Y: 0.435543, Z: 0.010384
[INFO] [1703829679.737494296] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829679.738045712] [move_group_interface]: Planning request accepted
[INFO] [1703829680.020812337] [move_group_interface]: Planning request complete!
[INFO] [1703829680.023238962] [move_group_interface]: time taken to generate plan: 0.110848 seconds
[INFO] [1703829680.023278921] [RTXAgentClientNode]: move_arm  took 286 ms
[INFO] [1703829680.024603546] [move_group_interface]: Execute request accepted
[INFO] [1703829721.350812468] [move_group_interface]: Execute request success!
[INFO] [1703829721.351698676] [RTXAgentClientNode]: Action took 42259 ms
[INFO] [1703829723.352085385] [RTXAgentClientNode]: *** Iteration 3
[INFO] [1703829723.352957885] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829723.929800969] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829723.929921636] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703829723.929973927] [RTXAgentClientNode]: Action server available after 577 ms
[ERROR] [1703829723.930003927] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829723.930010344] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703829723.930033969] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829723.930054844] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829723.930078636] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829723.930096677] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829723.930115302] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829723.930142552] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003857, Target: 0.000000
[INFO] [1703829723.930195136] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019433, Target: 0.019500
[INFO] [1703829723.930376261] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829723.951970219] [move_group_interface]: Planning request accepted
[INFO] [1703829724.086734302] [move_group_interface]: Planning request complete!
[INFO] [1703829724.087311552] [move_group_interface]: time taken to generate plan: 0.0328605 seconds
[INFO] [1703829724.087338094] [RTXAgentClientNode]: move_hand 157 ms
[INFO] [1703829724.088073302] [move_group_interface]: Execute request accepted
[INFO] [1703829724.142898844] [move_group_interface]: Execute request success!
[INFO] [1703829724.143694511] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703829724.143747094] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.223220, Y: -0.100893, Z: 0.455201, RPY --  R: 0.043582, P: 0.900279, Y: 0.043479, Q -- W: 0.900166, X: 0.010159, Y: 0.435312, Z: 0.010090
[INFO] [1703829724.143765344] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.223220, Y: -0.120893, Z: 0.475201, RPY --  R: 0.043582, P: 0.900279, Y: 0.043479, Q -- W: 0.899754, X: 0.029070, Y: 0.434459, Z: 0.029046
[INFO] [1703829724.143896427] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829724.144879844] [move_group_interface]: Planning request accepted
[INFO] [1703829724.345714761] [move_group_interface]: Planning request complete!
[INFO] [1703829724.346571386] [move_group_interface]: time taken to generate plan: 0.0451059 seconds
[INFO] [1703829724.346599219] [RTXAgentClientNode]: move_arm  took 202 ms
[INFO] [1703829724.347101844] [move_group_interface]: Execute request accepted
[INFO] [1703829729.219799930] [move_group_interface]: Execute request success!
[INFO] [1703829729.220142847] [RTXAgentClientNode]: Action took 5867 ms
[INFO] [1703829731.220867042] [RTXAgentClientNode]: *** Iteration 4
[INFO] [1703829731.222356375] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703829731.605315625] [RTXAgentClientNode]: Feedback callback
[INFO] [1703829731.605480208] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703829731.605567042] [RTXAgentClientNode]: Action server available after 384 ms
[ERROR] [1703829731.605621333] [RTXAgentClientNode]: move_hand began
[ERROR] [1703829731.605641542] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703829731.605662958] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703829731.605676417] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703829731.605689042] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703829731.605697958] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703829731.605733875] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003857, Target: 0.000000
[INFO] [1703829731.605748208] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019433, Target: 0.019500
[ERROR] [1703829731.605881708] [RTXAgentClientNode]: VLA error: 
[INFO] [1703829731.606075958] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829731.802319292] [move_group_interface]: Planning request accepted
[INFO] [1703829731.855814709] [move_group_interface]: Planning request complete!
[INFO] [1703829731.856546500] [move_group_interface]: time taken to generate plan: 0.0167637 seconds
[INFO] [1703829731.856575250] [RTXAgentClientNode]: move_hand 250 ms
[INFO] [1703829731.857145125] [move_group_interface]: Execute request accepted
[INFO] [1703829731.923776709] [move_group_interface]: Execute request success!
[INFO] [1703829731.923790292] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703829731.923830334] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.206141, Y: -0.123014, Z: 0.496707, RPY --  R: 0.124262, P: 0.894100, Y: 0.125149, Q -- W: 0.899904, X: 0.028898, Y: 0.434130, Z: 0.029489
[INFO] [1703829731.923842792] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.206141, Y: -0.143014, Z: 0.516707, RPY --  R: 0.124262, P: 0.894100, Y: 0.125149, Q -- W: 0.896546, X: 0.082861, Y: 0.427128, Z: 0.083069
[INFO] [1703829731.923994917] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703829731.925058125] [move_group_interface]: Planning request accepted
[INFO] [1703829732.098803459] [move_group_interface]: Planning request complete!
[INFO] [1703829732.099661000] [move_group_interface]: time taken to generate plan: 0.0173944 seconds
[INFO] [1703829732.099684709] [RTXAgentClientNode]: move_arm  took 175 ms
[INFO] [1703829732.100323125] [move_group_interface]: Execute request accepted
^C[INFO] [1703829736.358223877] [rclcpp]: signal_handler(signum=2)
^[[A^C[INFO] [1703830043.402232214] [rclcpp]: signal_handler(signum=2)
^C[INFO] [1703830044.273963673] [rclcpp]: signal_handler(signum=2)
Killed
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=10
[INFO] [1703831036.787068132] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0 seconds
[INFO] [1703831036.787115924] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703831036.856017299] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703831036.864007007] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703831036.867509924] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703831037.189402341] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0 seconds
[INFO] [1703831037.189426841] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703831037.254620882] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703831037.343742383] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703831037.344161008] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703831037.344174758] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703831037.344358174] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703831037.344551549] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703831037.349098508] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703831037.349131549] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703831037.381363466] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[INFO] [1703831037.381589008] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/attached_collision_object' for attached collision objects
[INFO] [1703831037.381599216] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703831037.381689674] [RTXAgentClientNode]: Waiting...
[WARN] [1703831037.382742299] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703831038.204656466] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831062.893872797] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831062.893974505] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831062.893985880] [RTXAgentClientNode]: Action server available after 25512 ms
[ERROR] [1703831062.894015505] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831062.894022172] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831062.894038380] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831062.894053589] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831062.894073297] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831062.894086422] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703831062.894030547] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831062.894929297] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[INFO] [1703831062.911759714] [RTXAgentClientNode]: HAND OPENNESS -- Current: -1.114286, Target: 0.000000
[INFO] [1703831062.911973422] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.000000, Target: 0.019500
[INFO] [1703831062.912093964] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831062.914193172] [move_group_interface]: Planning request accepted
[INFO] [1703831062.962681214] [move_group_interface]: Planning request complete!
[INFO] [1703831062.963289880] [move_group_interface]: time taken to generate plan: 0.0239588 seconds
[INFO] [1703831062.963442255] [RTXAgentClientNode]: move_hand 69 ms
[INFO] [1703831062.964331339] [move_group_interface]: Execute request accepted
[INFO] [1703831062.967643880] [move_group_interface]: Execute request aborted
[ERROR] [1703831062.968441214] [move_group_interface]: MoveGroupInterface::execute() failed or timeout reached
[INFO] [1703831062.968473880] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831062.968554339] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.306938, Y: -0.000000, Z: 0.244561, RPY --  R: -0.000000, P: 0.907151, Y: -0.000000, Q -- W: 0.898886, X: -0.000000, Y: 0.438182, Z: -0.000000
[INFO] [1703831062.968569047] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.306938, Y: -0.020000, Z: 0.264561, RPY --  R: -0.000000, P: 0.907151, Y: -0.000000, Q -- W: 0.898886, X: -0.000000, Y: 0.438182, Z: -0.000000
[INFO] [1703831062.968628672] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831062.972276880] [move_group_interface]: Planning request accepted
[INFO] [1703831063.040554547] [move_group_interface]: Planning request complete!
[INFO] [1703831063.040958547] [move_group_interface]: time taken to generate plan: 0.01515 seconds
[INFO] [1703831063.041020464] [RTXAgentClientNode]: move_arm  took 72 ms
[INFO] [1703831063.041544630] [move_group_interface]: Execute request accepted
[INFO] [1703831063.043181672] [move_group_interface]: Execute request aborted
[ERROR] [1703831063.044058505] [move_group_interface]: MoveGroupInterface::execute() failed or timeout reached
[INFO] [1703831063.044121630] [RTXAgentClientNode]: Action took 25662 ms
[INFO] [1703831063.044223505] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703831063.044990422] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831063.522420714] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831063.522522672] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[ERROR] [1703831063.522562589] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831063.522601756] [RTXAgentClientNode]: Action server available after 478 ms
[ERROR] [1703831063.522618339] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831063.522626214] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831063.522633214] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831063.522662256] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831063.522678714] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831063.522687381] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831063.522718881] [RTXAgentClientNode]: HAND OPENNESS -- Current: -1.114286, Target: 0.000000
[INFO] [1703831063.522730214] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.000000, Target: 0.019500
[INFO] [1703831063.522924672] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831063.523464256] [move_group_interface]: Planning request accepted
[INFO] [1703831063.573690964] [move_group_interface]: Planning request complete!
[INFO] [1703831063.573799089] [move_group_interface]: time taken to generate plan: 0.0270587 seconds
[INFO] [1703831063.573863089] [RTXAgentClientNode]: move_hand 51 ms
[INFO] [1703831063.575415506] [move_group_interface]: Execute request accepted
[INFO] [1703831082.123464500] [move_group_interface]: Execute request success!
[INFO] [1703831082.123961709] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831082.125001459] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.308721, Y: 0.000327, Z: 0.247229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: -0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703831082.125098625] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.308721, Y: -0.019673, Z: 0.267229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: 0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703831082.125337084] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831082.126563125] [move_group_interface]: Planning request accepted
[INFO] [1703831082.271183292] [move_group_interface]: Planning request complete!
[INFO] [1703831082.271661250] [move_group_interface]: time taken to generate plan: 0.0253363 seconds
[INFO] [1703831082.271710167] [RTXAgentClientNode]: move_arm  took 147 ms
[INFO] [1703831082.273460542] [move_group_interface]: Execute request accepted
[INFO] [1703831085.893569085] [move_group_interface]: Execute request success!
[INFO] [1703831085.894265669] [RTXAgentClientNode]: Action took 22850 ms
[INFO] [1703831085.894307502] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703831085.897945585] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831086.231550502] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831086.231615669] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831086.231673836] [RTXAgentClientNode]: Action server available after 337 ms
[ERROR] [1703831086.231718586] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831086.231749419] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831086.231774461] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831086.231813044] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831086.231820336] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703831086.231876711] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831086.231987336] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831086.232033877] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831086.232056377] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831086.232183586] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831086.232574461] [move_group_interface]: Planning request accepted
[INFO] [1703831086.275884002] [move_group_interface]: Planning request complete!
[INFO] [1703831086.276549002] [move_group_interface]: time taken to generate plan: 0.0117738 seconds
[INFO] [1703831086.276685252] [RTXAgentClientNode]: move_hand 44 ms
[INFO] [1703831086.277484752] [move_group_interface]: Execute request accepted
[INFO] [1703831086.384130794] [move_group_interface]: Execute request success!
[INFO] [1703831086.384961752] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831086.385079419] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291566, Y: -0.019580, Z: 0.288648, RPY --  R: 0.001015, P: 0.899071, Y: 0.000087, Q -- W: 0.900649, X: 0.000438, Y: 0.434547, Z: -0.000181
[INFO] [1703831086.385107877] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291566, Y: -0.039580, Z: 0.308648, RPY --  R: 0.001015, P: 0.899071, Y: 0.000087, Q -- W: 0.900649, X: 0.000476, Y: 0.434547, Z: 0.000259
[INFO] [1703831086.385300086] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831086.388086336] [move_group_interface]: Planning request accepted
[INFO] [1703831086.505320336] [move_group_interface]: Planning request aborted
[ERROR] [1703831086.505598711] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703831086.505623752] [RTXAgentClientNode]: move_arm  took 120 ms
[ERROR] [1703831086.505642086] [RTXAgentClientNode]: Planning failed!
[INFO] [1703831086.505650002] [RTXAgentClientNode]: Action took 611 ms
[INFO] [1703831086.505659544] [RTXAgentClientNode]: *** Iteration 3
[INFO] [1703831086.506293211] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831086.842745461] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831086.842771294] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831086.842912044] [RTXAgentClientNode]: Action server available after 337 ms
[ERROR] [1703831086.843005878] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831086.843056378] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831086.843101711] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831086.843133211] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703831086.843466919] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831086.843529419] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831086.843572378] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831086.843632044] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831086.843674128] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831086.843809419] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831086.844517294] [move_group_interface]: Planning request accepted
[INFO] [1703831086.897198253] [move_group_interface]: Planning request complete!
[INFO] [1703831086.897645169] [move_group_interface]: time taken to generate plan: 0.0141594 seconds
[INFO] [1703831086.897671878] [RTXAgentClientNode]: move_hand 54 ms
[INFO] [1703831086.898180211] [move_group_interface]: Execute request accepted
[INFO] [1703831086.950337753] [move_group_interface]: Execute request success!
[INFO] [1703831086.951264378] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831086.951462669] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291508, Y: -0.019612, Z: 0.288756, RPY --  R: 0.001035, P: 0.899024, Y: 0.000195, Q -- W: 0.900659, X: 0.000424, Y: 0.434526, Z: -0.000137
[INFO] [1703831086.951515711] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291508, Y: -0.039612, Z: 0.308756, RPY --  R: 0.001035, P: 0.899024, Y: 0.000195, Q -- W: 0.900659, X: 0.000509, Y: 0.434526, Z: 0.000313
[INFO] [1703831086.951630003] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831086.952893128] [move_group_interface]: Planning request accepted
[INFO] [1703831087.052449378] [move_group_interface]: Planning request aborted
[ERROR] [1703831087.052655711] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703831087.052680919] [RTXAgentClientNode]: move_arm  took 101 ms
[ERROR] [1703831087.052701336] [RTXAgentClientNode]: Planning failed!
[INFO] [1703831087.052715336] [RTXAgentClientNode]: Action took 546 ms
[INFO] [1703831087.052723544] [RTXAgentClientNode]: *** Iteration 4
[INFO] [1703831087.053282003] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831087.376403836] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831087.376463711] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831087.376474753] [RTXAgentClientNode]: Action server available after 323 ms
[ERROR] [1703831087.376546044] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831087.376584753] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703831087.376777211] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831087.376834878] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831087.376872794] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831087.376924919] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831087.376961586] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831087.377021461] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831087.377067961] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831087.377517169] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831087.378409169] [move_group_interface]: Planning request accepted
[INFO] [1703831087.450860461] [move_group_interface]: Planning request complete!
[INFO] [1703831087.451444919] [move_group_interface]: time taken to generate plan: 0.0128212 seconds
[INFO] [1703831087.451477294] [RTXAgentClientNode]: move_hand 74 ms
[INFO] [1703831087.451890336] [move_group_interface]: Execute request accepted
[INFO] [1703831087.504071794] [move_group_interface]: Execute request success!
[INFO] [1703831087.504929544] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831087.505016461] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291489, Y: -0.019622, Z: 0.288791, RPY --  R: 0.001042, P: 0.899009, Y: 0.000231, Q -- W: 0.900662, X: 0.000419, Y: 0.434519, Z: -0.000123
[INFO] [1703831087.505029211] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291489, Y: -0.039622, Z: 0.308791, RPY --  R: 0.001042, P: 0.899009, Y: 0.000231, Q -- W: 0.900662, X: 0.000519, Y: 0.434519, Z: 0.000330
[INFO] [1703831087.505133128] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831087.505644211] [move_group_interface]: Planning request accepted
[INFO] [1703831088.455898128] [move_group_interface]: Planning request aborted
[ERROR] [1703831088.456429337] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703831088.456708628] [RTXAgentClientNode]: move_arm  took 951 ms
[ERROR] [1703831088.457077712] [RTXAgentClientNode]: Planning failed!
[INFO] [1703831088.457278712] [RTXAgentClientNode]: Action took 1404 ms
[INFO] [1703831088.457347253] [RTXAgentClientNode]: *** Iteration 5
[INFO] [1703831088.457902795] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831088.749733420] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831088.749857753] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831088.749911670] [RTXAgentClientNode]: Action server available after 292 ms
[ERROR] [1703831088.750039087] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703831088.752603795] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831088.752753545] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831088.752820670] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831088.752839337] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831088.752856045] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831088.752873378] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831088.752942337] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831088.752979753] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831088.753119503] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831088.754141087] [move_group_interface]: Planning request accepted
[INFO] [1703831088.820949337] [move_group_interface]: Planning request complete!
[INFO] [1703831088.821739087] [move_group_interface]: time taken to generate plan: 0.0125518 seconds
[INFO] [1703831088.821897837] [RTXAgentClientNode]: move_hand 68 ms
[INFO] [1703831088.822529462] [move_group_interface]: Execute request accepted
[INFO] [1703831088.884605753] [move_group_interface]: Execute request success!
[INFO] [1703831088.885320170] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831088.885495170] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291482, Y: -0.019626, Z: 0.288804, RPY --  R: 0.001045, P: 0.899003, Y: 0.000244, Q -- W: 0.900664, X: 0.000418, Y: 0.434517, Z: -0.000117
[INFO] [1703831088.885536295] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291482, Y: -0.039626, Z: 0.308804, RPY --  R: 0.001045, P: 0.899003, Y: 0.000244, Q -- W: 0.900664, X: 0.000524, Y: 0.434516, Z: 0.000337
[INFO] [1703831088.885739795] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831088.890594378] [move_group_interface]: Planning request accepted
[INFO] [1703831089.019545170] [move_group_interface]: Planning request aborted
[ERROR] [1703831089.020018212] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703831089.020050254] [RTXAgentClientNode]: move_arm  took 134 ms
[ERROR] [1703831089.020073629] [RTXAgentClientNode]: Planning failed!
[INFO] [1703831089.020084629] [RTXAgentClientNode]: Action took 562 ms
[INFO] [1703831089.020092212] [RTXAgentClientNode]: *** Iteration 6
[INFO] [1703831089.020523962] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831089.300832337] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831089.300901129] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[ERROR] [1703831089.300999462] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831089.301364379] [RTXAgentClientNode]: Action server available after 281 ms
[ERROR] [1703831089.301382129] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831089.301386920] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831089.301392045] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831089.301396129] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831089.301399795] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831089.301403629] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831089.301417004] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831089.301422004] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831089.301524212] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831089.302590754] [move_group_interface]: Planning request accepted
[INFO] [1703831089.352927504] [move_group_interface]: Planning request complete!
[INFO] [1703831089.355704879] [move_group_interface]: time taken to generate plan: 0.0146952 seconds
[INFO] [1703831089.355731545] [RTXAgentClientNode]: move_hand 54 ms
[INFO] [1703831089.356324920] [move_group_interface]: Execute request accepted
[INFO] [1703831089.410599712] [move_group_interface]: Execute request success!
[INFO] [1703831089.410672920] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831089.410738545] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291482, Y: -0.019627, Z: 0.288805, RPY --  R: 0.001045, P: 0.899003, Y: 0.000244, Q -- W: 0.900664, X: 0.000417, Y: 0.434516, Z: -0.000117
[INFO] [1703831089.410758795] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291482, Y: -0.039627, Z: 0.308805, RPY --  R: 0.001045, P: 0.899003, Y: 0.000244, Q -- W: 0.900664, X: 0.000524, Y: 0.434516, Z: 0.000337
[INFO] [1703831089.410853879] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831089.411190379] [move_group_interface]: Planning request accepted
[INFO] [1703831089.558616420] [move_group_interface]: Planning request aborted
[ERROR] [1703831089.558824587] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703831089.558844962] [RTXAgentClientNode]: move_arm  took 148 ms
[ERROR] [1703831089.558858295] [RTXAgentClientNode]: Planning failed!
[INFO] [1703831089.558869087] [RTXAgentClientNode]: Action took 538 ms
[INFO] [1703831089.558875670] [RTXAgentClientNode]: *** Iteration 7
[INFO] [1703831089.559596379] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831089.797046837] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831089.797121921] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831089.797149421] [RTXAgentClientNode]: Action server available after 238 ms
[ERROR] [1703831089.797212212] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831089.797316004] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831089.797373087] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831089.797412212] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831089.797480129] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831089.797517129] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831089.797578629] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831089.797632671] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[ERROR] [1703831089.797212629] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831089.797755712] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831089.798880337] [move_group_interface]: Planning request accepted
[INFO] [1703831089.844092254] [move_group_interface]: Planning request complete!
[INFO] [1703831089.845119462] [move_group_interface]: time taken to generate plan: 0.0137048 seconds
[INFO] [1703831089.845152921] [RTXAgentClientNode]: move_hand 47 ms
[INFO] [1703831089.845706754] [move_group_interface]: Execute request accepted
[INFO] [1703831089.897598796] [move_group_interface]: Execute request success!
[INFO] [1703831089.898950379] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831089.899015504] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291482, Y: -0.019627, Z: 0.288805, RPY --  R: 0.001045, P: 0.899003, Y: 0.000245, Q -- W: 0.900664, X: 0.000417, Y: 0.434516, Z: -0.000117
[INFO] [1703831089.899028587] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291482, Y: -0.039627, Z: 0.308805, RPY --  R: 0.001045, P: 0.899003, Y: 0.000245, Q -- W: 0.900664, X: 0.000524, Y: 0.434516, Z: 0.000337
[INFO] [1703831089.899161629] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831089.902032921] [move_group_interface]: Planning request accepted
[INFO] [1703831090.009477087] [move_group_interface]: Planning request aborted
[ERROR] [1703831090.009861087] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703831090.009883171] [RTXAgentClientNode]: move_arm  took 110 ms
[ERROR] [1703831090.009901504] [RTXAgentClientNode]: Planning failed!
[INFO] [1703831090.009906879] [RTXAgentClientNode]: Action took 451 ms
[INFO] [1703831090.009913087] [RTXAgentClientNode]: *** Iteration 8
[INFO] [1703831090.010561504] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831090.374574796] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831090.374700504] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831090.374766296] [RTXAgentClientNode]: Action server available after 364 ms
[ERROR] [1703831090.374849379] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831090.374876504] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831090.374895588] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831090.374907838] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831090.374923504] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831090.374940879] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831090.374980838] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831090.374996171] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[ERROR] [1703831090.375100546] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831090.375246879] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831091.231720963] [move_group_interface]: Planning request accepted
[INFO] [1703831091.297310296] [move_group_interface]: Planning request complete!
[INFO] [1703831091.297711880] [move_group_interface]: time taken to generate plan: 0.023415 seconds
[INFO] [1703831091.297733713] [RTXAgentClientNode]: move_hand 922 ms
[INFO] [1703831091.301966005] [move_group_interface]: Execute request accepted
[INFO] [1703831091.354293171] [move_group_interface]: Execute request success!
[INFO] [1703831091.357141421] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831091.357264546] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291482, Y: -0.019627, Z: 0.288805, RPY --  R: 0.001045, P: 0.899003, Y: 0.000245, Q -- W: 0.900664, X: 0.000417, Y: 0.434516, Z: -0.000117
[INFO] [1703831091.357310255] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291482, Y: -0.039627, Z: 0.308805, RPY --  R: 0.001045, P: 0.899003, Y: 0.000245, Q -- W: 0.900664, X: 0.000524, Y: 0.434516, Z: 0.000337
[INFO] [1703831091.357675921] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831091.358033130] [move_group_interface]: Planning request accepted
[INFO] [1703831091.475816088] [move_group_interface]: Planning request aborted
[ERROR] [1703831091.476442963] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703831091.476465213] [RTXAgentClientNode]: move_arm  took 119 ms
[ERROR] [1703831091.476487046] [RTXAgentClientNode]: Planning failed!
[INFO] [1703831091.476495880] [RTXAgentClientNode]: Action took 1466 ms
[INFO] [1703831091.476504546] [RTXAgentClientNode]: *** Iteration 9
[INFO] [1703831091.477238796] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831091.794177547] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831091.794211630] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831091.794270755] [RTXAgentClientNode]: Action server available after 317 ms
[ERROR] [1703831091.794290922] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831091.794303005] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831091.794313380] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831091.794322172] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831091.794326088] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831091.794330505] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831091.794343838] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831091.794355338] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[ERROR] [1703831091.794305338] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831091.794472672] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831091.795193338] [move_group_interface]: Planning request accepted
[INFO] [1703831091.829236297] [move_group_interface]: Planning request complete!
[INFO] [1703831091.829757005] [move_group_interface]: time taken to generate plan: 0.00369696 seconds
[INFO] [1703831091.829784713] [RTXAgentClientNode]: move_hand 35 ms
[INFO] [1703831091.830130130] [move_group_interface]: Execute request accepted
[INFO] [1703831091.881646172] [move_group_interface]: Execute request success!
[INFO] [1703831091.881753130] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831091.881913130] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291482, Y: -0.019627, Z: 0.288805, RPY --  R: 0.001045, P: 0.899003, Y: 0.000245, Q -- W: 0.900664, X: 0.000417, Y: 0.434516, Z: -0.000117
[INFO] [1703831091.881948213] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291482, Y: -0.039627, Z: 0.308805, RPY --  R: 0.001045, P: 0.899003, Y: 0.000245, Q -- W: 0.900664, X: 0.000524, Y: 0.434516, Z: 0.000337
[INFO] [1703831091.882145297] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831091.883191963] [move_group_interface]: Planning request accepted
[INFO] [1703831092.047638838] [move_group_interface]: Planning request aborted
[ERROR] [1703831092.048536713] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703831092.048566088] [RTXAgentClientNode]: move_arm  took 166 ms
[ERROR] [1703831092.048583630] [RTXAgentClientNode]: Planning failed!
[INFO] [1703831092.048591588] [RTXAgentClientNode]: Action took 572 ms
[INFO] [1703831092.055605838] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[INFO] [1703831092.056054880] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
double free or corruption (!prev)
[ros2run]: Aborted
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=10
[INFO] [1703831132.012096301] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 34.52 seconds
[INFO] [1703831132.012383385] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703831132.131081051] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703831132.157661926] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703831132.161704301] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703831132.518445093] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.2 seconds
[INFO] [1703831132.518504302] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703831132.634340593] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703831132.766185302] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703831132.766727968] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703831132.766770427] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703831132.766984135] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703831132.767231052] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703831132.776032260] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703831132.776058635] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703831132.822466927] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[INFO] [1703831132.825365718] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/attached_collision_object' for attached collision objects
[INFO] [1703831132.825386635] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703831132.825529760] [RTXAgentClientNode]: Waiting...
[INFO] [1703831132.826154260] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831133.127808302] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831133.127882010] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831133.128089844] [RTXAgentClientNode]: Action server available after 302 ms
[ERROR] [1703831133.128264927] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831133.128278677] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703831133.128419344] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831133.128473219] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831133.128515385] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831133.128569969] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831133.128575052] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831133.129302010] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[INFO] [1703831133.132703219] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831133.132851344] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831133.133008177] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831133.133504010] [move_group_interface]: Planning request accepted
[INFO] [1703831133.204775469] [move_group_interface]: Planning request complete!
[INFO] [1703831133.205508177] [move_group_interface]: time taken to generate plan: 0.0130871 seconds
[INFO] [1703831133.205535760] [RTXAgentClientNode]: move_hand 76 ms
[INFO] [1703831133.206409469] [move_group_interface]: Execute request accepted
[INFO] [1703831133.264333344] [move_group_interface]: Execute request success!
[INFO] [1703831133.264684719] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831133.264832344] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291482, Y: -0.019627, Z: 0.288805, RPY --  R: 0.001045, P: 0.899003, Y: 0.000245, Q -- W: 0.900664, X: 0.000417, Y: 0.434516, Z: -0.000117
[INFO] [1703831133.264853302] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291482, Y: -0.039627, Z: 0.308805, RPY --  R: 0.001045, P: 0.899003, Y: 0.000245, Q -- W: 0.900664, X: 0.000524, Y: 0.434516, Z: 0.000337
[INFO] [1703831133.264940885] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831133.266522469] [move_group_interface]: Planning request accepted
[INFO] [1703831133.412571135] [move_group_interface]: Planning request aborted
[ERROR] [1703831133.412692469] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703831133.412768510] [RTXAgentClientNode]: move_arm  took 148 ms
[ERROR] [1703831133.412825760] [RTXAgentClientNode]: Planning failed!
[INFO] [1703831133.412850677] [RTXAgentClientNode]: Action took 587 ms
[INFO] [1703831133.412875677] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703831133.413471427] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831133.716437511] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831133.716504594] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831133.716532427] [RTXAgentClientNode]: Action server available after 303 ms
[ERROR] [1703831133.716570427] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831133.716580177] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831133.716587094] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831133.716595844] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831133.716602594] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831133.716610511] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703831133.716617261] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831133.716677969] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831133.716695677] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831133.716782011] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831133.717198386] [move_group_interface]: Planning request accepted
[INFO] [1703831133.772168636] [move_group_interface]: Planning request complete!
[INFO] [1703831133.772213136] [move_group_interface]: time taken to generate plan: 0.0121107 seconds
[INFO] [1703831133.772235886] [RTXAgentClientNode]: move_hand 55 ms
[INFO] [1703831133.772700052] [move_group_interface]: Execute request accepted
[INFO] [1703831133.830297344] [move_group_interface]: Execute request success!
[INFO] [1703831133.831031261] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831133.831295969] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291482, Y: -0.019627, Z: 0.288805, RPY --  R: 0.001045, P: 0.899003, Y: 0.000245, Q -- W: 0.900664, X: 0.000417, Y: 0.434516, Z: -0.000117
[INFO] [1703831133.831366677] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291482, Y: -0.039627, Z: 0.308805, RPY --  R: 0.001045, P: 0.899003, Y: 0.000245, Q -- W: 0.900664, X: 0.000524, Y: 0.434516, Z: 0.000337
[INFO] [1703831133.831451677] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831133.832072177] [move_group_interface]: Planning request accepted
[INFO] [1703831133.957997594] [move_group_interface]: Planning request aborted
[ERROR] [1703831133.958759927] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703831133.958793469] [RTXAgentClientNode]: move_arm  took 127 ms
[ERROR] [1703831133.958824302] [RTXAgentClientNode]: Planning failed!
[INFO] [1703831133.958835219] [RTXAgentClientNode]: Action took 545 ms
[INFO] [1703831133.958843094] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703831133.959612969] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831134.229972844] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831134.230061802] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831134.230083344] [RTXAgentClientNode]: Action server available after 271 ms
[ERROR] [1703831134.230104177] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831134.230139011] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831134.230146886] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831134.230147177] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703831134.230179261] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831134.230192844] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831134.230197386] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831134.230211761] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831134.230217011] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831134.230312261] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831134.380274886] [move_group_interface]: Planning request accepted
[INFO] [1703831134.438978553] [move_group_interface]: Planning request complete!
[INFO] [1703831134.439708469] [move_group_interface]: time taken to generate plan: 0.0148891 seconds
[INFO] [1703831134.439768594] [RTXAgentClientNode]: move_hand 209 ms
[INFO] [1703831134.440555886] [move_group_interface]: Execute request accepted
[INFO] [1703831134.494862969] [move_group_interface]: Execute request success!
[INFO] [1703831134.495267428] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831134.495336844] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291482, Y: -0.019627, Z: 0.288805, RPY --  R: 0.001045, P: 0.899003, Y: 0.000245, Q -- W: 0.900664, X: 0.000417, Y: 0.434516, Z: -0.000117
[INFO] [1703831134.495355136] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291482, Y: -0.039627, Z: 0.308805, RPY --  R: 0.001045, P: 0.899003, Y: 0.000245, Q -- W: 0.900664, X: 0.000524, Y: 0.434516, Z: 0.000337
[INFO] [1703831134.495436386] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831134.495702594] [move_group_interface]: Planning request accepted
[INFO] [1703831134.605386011] [move_group_interface]: Planning request complete!
[INFO] [1703831134.605805219] [move_group_interface]: time taken to generate plan: 0.0292309 seconds
[INFO] [1703831134.605841136] [RTXAgentClientNode]: move_arm  took 110 ms
[INFO] [1703831134.607935428] [move_group_interface]: Execute request accepted
[INFO] [1703831138.904560388] [move_group_interface]: Execute request success!
[INFO] [1703831138.907674930] [RTXAgentClientNode]: Action took 4948 ms
[INFO] [1703831138.907891555] [RTXAgentClientNode]: *** Iteration 3
[INFO] [1703831138.909620805] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831139.223938055] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831139.223977388] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831139.223989347] [RTXAgentClientNode]: Action server available after 316 ms
[ERROR] [1703831139.224011930] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831139.224017513] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831139.224023013] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831139.224027097] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831139.224030847] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831139.224035888] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831139.224056472] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831139.224071972] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831139.224200763] [move_group_interface]: MoveGroup action client/server ready
[ERROR] [1703831139.224401388] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831139.224492597] [move_group_interface]: Planning request accepted
[INFO] [1703831139.262862305] [move_group_interface]: Planning request complete!
[INFO] [1703831139.265648430] [move_group_interface]: time taken to generate plan: 0.00468458 seconds
[INFO] [1703831139.265688013] [RTXAgentClientNode]: move_hand 41 ms
[INFO] [1703831139.266221597] [move_group_interface]: Execute request accepted
[INFO] [1703831139.332831388] [move_group_interface]: Execute request success!
[INFO] [1703831139.333403638] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831139.333539888] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.274705, Y: -0.039574, Z: 0.330097, RPY --  R: 0.002472, P: 0.898039, Y: 0.001843, Q -- W: 0.900873, X: 0.000713, Y: 0.434083, Z: 0.000294
[INFO] [1703831139.333618680] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.274705, Y: -0.059574, Z: 0.350097, RPY --  R: 0.002472, P: 0.898039, Y: 0.001843, Q -- W: 0.900872, X: 0.001513, Y: 0.434081, Z: 0.001367
[INFO] [1703831139.333734263] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831139.334462055] [move_group_interface]: Planning request accepted
[INFO] [1703831139.453828180] [move_group_interface]: Planning request complete!
[INFO] [1703831139.461150680] [move_group_interface]: time taken to generate plan: 0.0285939 seconds
[INFO] [1703831139.461186763] [RTXAgentClientNode]: move_arm  took 127 ms
[INFO] [1703831139.461651430] [move_group_interface]: Execute request accepted
[INFO] [1703831142.853244584] [move_group_interface]: Execute request success!
[INFO] [1703831142.853851917] [RTXAgentClientNode]: Action took 3946 ms
[INFO] [1703831142.853891167] [RTXAgentClientNode]: *** Iteration 4
[INFO] [1703831142.854756917] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831143.226390292] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831143.226448084] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831143.226661542] [RTXAgentClientNode]: Action server available after 372 ms
[ERROR] [1703831143.226700376] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831143.226707084] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831143.226713376] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831143.226718042] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831143.226741042] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831143.226754501] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831143.226796917] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831143.226811917] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831143.226915542] [move_group_interface]: MoveGroup action client/server ready
[ERROR] [1703831143.226917501] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831143.228257084] [move_group_interface]: Planning request accepted
[INFO] [1703831143.269070167] [move_group_interface]: Planning request complete!
[INFO] [1703831143.270391209] [move_group_interface]: time taken to generate plan: 0.0134858 seconds
[INFO] [1703831143.270707376] [RTXAgentClientNode]: move_hand 43 ms
[INFO] [1703831143.271597376] [move_group_interface]: Execute request accepted
[INFO] [1703831143.323778626] [move_group_interface]: Execute request success!
[INFO] [1703831143.324378584] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831143.324448293] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.257761, Y: -0.059541, Z: 0.371349, RPY --  R: 0.006152, P: 0.898007, Y: 0.004535, Q -- W: 0.900877, X: 0.001787, Y: 0.434071, Z: 0.000708
[INFO] [1703831143.324476918] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.257761, Y: -0.079541, Z: 0.391349, RPY --  R: 0.006152, P: 0.898007, Y: 0.004535, Q -- W: 0.900871, X: 0.003755, Y: 0.434058, Z: 0.003378
[INFO] [1703831143.324566376] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831143.325005501] [move_group_interface]: Planning request accepted
[INFO] [1703831143.433784626] [move_group_interface]: Planning request complete!
[INFO] [1703831143.434067626] [move_group_interface]: time taken to generate plan: 0.0182782 seconds
[INFO] [1703831143.434098751] [RTXAgentClientNode]: move_arm  took 109 ms
[INFO] [1703831143.434934376] [move_group_interface]: Execute request accepted
[INFO] [1703831146.778014794] [move_group_interface]: Execute request success!
[INFO] [1703831146.778922044] [RTXAgentClientNode]: Action took 3925 ms
[INFO] [1703831146.779114752] [RTXAgentClientNode]: *** Iteration 5
[INFO] [1703831146.779437586] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831147.059889211] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831147.059993586] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831147.060011336] [RTXAgentClientNode]: Action server available after 280 ms
[ERROR] [1703831147.060049378] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831147.060055253] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831147.060061461] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831147.060065628] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831147.060069294] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831147.060074586] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703831147.060171669] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831147.079487211] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831147.079526794] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831147.079597128] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831147.082388919] [move_group_interface]: Planning request accepted
[INFO] [1703831147.131363961] [move_group_interface]: Planning request complete!
[INFO] [1703831147.132021169] [move_group_interface]: time taken to generate plan: 0.0120655 seconds
[INFO] [1703831147.132048503] [RTXAgentClientNode]: move_hand 71 ms
[INFO] [1703831147.132379211] [move_group_interface]: Execute request accepted
[INFO] [1703831147.184612753] [move_group_interface]: Execute request success!
[INFO] [1703831147.184702294] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831147.185074711] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.240780, Y: -0.079706, Z: 0.412624, RPY --  R: 0.014001, P: 0.898131, Y: 0.013486, Q -- W: 0.900831, X: 0.003379, Y: 0.434146, Z: 0.003035
[INFO] [1703831147.185114753] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.240780, Y: -0.099706, Z: 0.432624, RPY --  R: 0.014001, P: 0.898131, Y: 0.013486, Q -- W: 0.900790, X: 0.009233, Y: 0.434061, Z: 0.009113
[INFO] [1703831147.185198753] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831147.186312128] [move_group_interface]: Planning request accepted
[INFO] [1703831147.405724878] [move_group_interface]: Planning request aborted
[ERROR] [1703831147.405953419] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703831147.405999336] [RTXAgentClientNode]: move_arm  took 221 ms
[ERROR] [1703831147.406024294] [RTXAgentClientNode]: Planning failed!
[INFO] [1703831147.406031669] [RTXAgentClientNode]: Action took 626 ms
[INFO] [1703831147.406045461] [RTXAgentClientNode]: *** Iteration 6
[INFO] [1703831147.407090336] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831147.670008545] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831147.670223836] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831147.670277711] [RTXAgentClientNode]: Action server available after 264 ms
[ERROR] [1703831147.670335211] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831147.670379420] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831147.670476045] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831147.670503128] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831147.670547086] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831147.670644170] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831147.670698086] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831147.670741878] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831147.670846753] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831147.671308086] [move_group_interface]: Planning request accepted
[ERROR] [1703831147.674175086] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831147.718300795] [move_group_interface]: Planning request complete!
[INFO] [1703831147.719309295] [move_group_interface]: time taken to generate plan: 0.011683 seconds
[INFO] [1703831147.719335503] [RTXAgentClientNode]: move_hand 48 ms
[INFO] [1703831147.720128045] [move_group_interface]: Execute request accepted
[INFO] [1703831147.772979461] [move_group_interface]: Execute request success!
[INFO] [1703831147.774476503] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831147.774531170] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.240677, Y: -0.079790, Z: 0.412833, RPY --  R: 0.013922, P: 0.898086, Y: 0.013523, Q -- W: 0.900841, X: 0.003336, Y: 0.434126, Z: 0.003069
[INFO] [1703831147.774543211] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.240677, Y: -0.099790, Z: 0.432833, RPY --  R: 0.013922, P: 0.898086, Y: 0.013523, Q -- W: 0.900800, X: 0.009206, Y: 0.434041, Z: 0.009113
[INFO] [1703831147.774603795] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831147.774932545] [move_group_interface]: Planning request accepted
[INFO] [1703831147.915336753] [move_group_interface]: Planning request aborted
[ERROR] [1703831147.915847878] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703831147.915880586] [RTXAgentClientNode]: move_arm  took 141 ms
[ERROR] [1703831147.915913711] [RTXAgentClientNode]: Planning failed!
[INFO] [1703831147.915932045] [RTXAgentClientNode]: Action took 509 ms
[INFO] [1703831147.915952586] [RTXAgentClientNode]: *** Iteration 7
[INFO] [1703831147.916825086] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831148.153021086] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831148.153062961] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831148.153102586] [RTXAgentClientNode]: Action server available after 237 ms
[ERROR] [1703831148.153152128] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703831148.153194295] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831148.153200128] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831148.153212086] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831148.153217753] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831148.153223003] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831148.153229086] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831148.153253253] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831148.153261920] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831148.153385045] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831148.154226003] [move_group_interface]: Planning request accepted
[INFO] [1703831148.183519461] [move_group_interface]: Planning request complete!
[INFO] [1703831148.184398086] [move_group_interface]: time taken to generate plan: 0.00422542 seconds
[INFO] [1703831148.184649086] [RTXAgentClientNode]: move_hand 31 ms
[INFO] [1703831148.186945545] [move_group_interface]: Execute request accepted
[INFO] [1703831148.243229920] [move_group_interface]: Execute request success!
[INFO] [1703831148.243998753] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831148.244054670] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.240653, Y: -0.079809, Z: 0.412881, RPY --  R: 0.013904, P: 0.898076, Y: 0.013532, Q -- W: 0.900843, X: 0.003325, Y: 0.434121, Z: 0.003077
[INFO] [1703831148.244074837] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.240653, Y: -0.099809, Z: 0.432881, RPY --  R: 0.013904, P: 0.898076, Y: 0.013532, Q -- W: 0.900802, X: 0.009199, Y: 0.434036, Z: 0.009113
[INFO] [1703831148.244144878] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831148.245101295] [move_group_interface]: Planning request accepted
[INFO] [1703831148.418731587] [move_group_interface]: Planning request complete!
[INFO] [1703831148.418947170] [move_group_interface]: time taken to generate plan: 0.0513932 seconds
[INFO] [1703831148.419036878] [RTXAgentClientNode]: move_arm  took 175 ms
[INFO] [1703831148.420582503] [move_group_interface]: Execute request accepted
[INFO] [1703831181.888727922] [move_group_interface]: Execute request success!
[INFO] [1703831181.889023797] [RTXAgentClientNode]: Action took 33975 ms
[INFO] [1703831181.889068797] [RTXAgentClientNode]: *** Iteration 8
[INFO] [1703831181.889619672] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831182.242235880] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831182.242389005] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831182.242451463] [RTXAgentClientNode]: Action server available after 353 ms
[ERROR] [1703831182.242502922] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703831182.242850047] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831182.242857672] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831182.243619755] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831182.243634547] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831182.243640172] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831182.243645047] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831182.243660005] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831182.243673255] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831182.243750880] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831182.244591797] [move_group_interface]: Planning request accepted
[INFO] [1703831182.386318880] [move_group_interface]: Planning request complete!
[INFO] [1703831182.387018463] [move_group_interface]: time taken to generate plan: 0.0142381 seconds
[INFO] [1703831182.387072630] [RTXAgentClientNode]: move_hand 143 ms
[INFO] [1703831182.387849297] [move_group_interface]: Execute request accepted
[INFO] [1703831182.442282714] [move_group_interface]: Execute request success!
[INFO] [1703831182.442573630] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831182.442712589] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.223415, Y: -0.100367, Z: 0.454509, RPY --  R: 0.038049, P: 0.897621, Y: 0.040319, Q -- W: 0.900784, X: 0.008391, Y: 0.434073, Z: 0.009906
[INFO] [1703831182.442766172] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.223415, Y: -0.120367, Z: 0.474509, RPY --  R: 0.038049, P: 0.897621, Y: 0.040319, Q -- W: 0.900451, X: 0.025881, Y: 0.433382, Z: 0.026411
[INFO] [1703831182.442961339] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831182.443935214] [move_group_interface]: Planning request accepted
[INFO] [1703831182.564371089] [move_group_interface]: Planning request complete!
[INFO] [1703831182.565076005] [move_group_interface]: time taken to generate plan: 0.0184521 seconds
[INFO] [1703831182.565131047] [RTXAgentClientNode]: move_arm  took 122 ms
[INFO] [1703831182.565664047] [move_group_interface]: Execute request accepted
[INFO] [1703831186.062032132] [move_group_interface]: Execute request success!
[INFO] [1703831186.063031382] [RTXAgentClientNode]: Action took 4173 ms
[INFO] [1703831186.063099507] [RTXAgentClientNode]: *** Iteration 9
[INFO] [1703831186.068609174] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831186.432542757] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831186.432747549] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[ERROR] [1703831186.432856882] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831186.433062090] [RTXAgentClientNode]: Action server available after 369 ms
[ERROR] [1703831186.433076340] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831186.433086382] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831186.433095299] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831186.433103507] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831186.433118965] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831186.433127840] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831186.438255424] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831186.438302965] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831186.438490799] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831186.439327924] [move_group_interface]: Planning request accepted
[INFO] [1703831186.476430715] [move_group_interface]: Planning request complete!
[INFO] [1703831186.477420382] [move_group_interface]: time taken to generate plan: 0.011927 seconds
[INFO] [1703831186.477439590] [RTXAgentClientNode]: move_hand 44 ms
[INFO] [1703831186.477823465] [move_group_interface]: Execute request accepted
[INFO] [1703831186.537665215] [move_group_interface]: Execute request success!
[INFO] [1703831186.538556132] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831186.538630090] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.206384, Y: -0.122135, Z: 0.495793, RPY --  R: 0.109541, P: 0.893272, Y: 0.110588, Q -- W: 0.900483, X: 0.025462, Y: 0.433356, Z: 0.026161
[INFO] [1703831186.538644424] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.206384, Y: -0.142135, Z: 0.515793, RPY --  R: 0.109541, P: 0.893272, Y: 0.110588, Q -- W: 0.897870, X: 0.073133, Y: 0.427898, Z: 0.073379
[INFO] [1703831186.538716424] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831186.539550590] [move_group_interface]: Planning request accepted
[INFO] [1703831186.650348799] [move_group_interface]: Planning request complete!
[INFO] [1703831186.650744799] [move_group_interface]: time taken to generate plan: 0.0292516 seconds
[INFO] [1703831186.650825132] [RTXAgentClientNode]: move_arm  took 112 ms
[INFO] [1703831186.652544757] [move_group_interface]: Execute request accepted
[INFO] [1703831190.789214717] [move_group_interface]: Execute request success!
[INFO] [1703831190.790148592] [RTXAgentClientNode]: Action took 4727 ms
[INFO] [1703831190.795164717] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[INFO] [1703831190.795330884] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[ros2run]: Segmentation fault
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=10
[INFO] [1703831219.426973467] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 78.9 seconds
[INFO] [1703831219.427066800] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703831219.546249134] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703831219.564783550] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703831219.573260842] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703831219.822739175] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.16 seconds
[INFO] [1703831219.822932884] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703831219.964772884] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703831220.089685759] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703831220.090063342] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703831220.090070842] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703831220.090193675] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703831220.090315925] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703831220.098666925] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703831220.098716884] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703831220.155365259] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[INFO] [1703831220.155675092] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/attached_collision_object' for attached collision objects
[INFO] [1703831220.155762926] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703831220.155906176] [RTXAgentClientNode]: Waiting...
[INFO] [1703831220.156121884] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831220.463870384] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831220.463989384] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831220.464099801] [RTXAgentClientNode]: Action server available after 308 ms
[ERROR] [1703831220.464279967] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703831220.464482717] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831220.464536259] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831220.464577426] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831220.464629926] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831220.464668926] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831220.464717467] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831220.465498217] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703831220.466716551] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703831220.494795759] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831220.494828384] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831220.494933967] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831220.495412759] [move_group_interface]: Planning request accepted
[INFO] [1703831220.541469801] [move_group_interface]: Planning request complete!
[INFO] [1703831220.542417884] [move_group_interface]: time taken to generate plan: 0.00250067 seconds
[INFO] [1703831220.542492384] [RTXAgentClientNode]: move_hand 77 ms
[INFO] [1703831220.547571884] [move_group_interface]: Execute request accepted
[INFO] [1703831220.595499592] [move_group_interface]: Execute request success!
[INFO] [1703831220.597409301] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831220.597472509] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.189211, Y: -0.147426, Z: 0.536631, RPY --  R: 0.302313, P: 0.858997, Y: 0.303164, Q -- W: 0.897970, X: 0.073173, Y: 0.427619, Z: 0.073737
[INFO] [1703831220.597488801] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.189211, Y: -0.167426, Z: 0.556631, RPY --  R: 0.302313, P: 0.858997, Y: 0.303164, Q -- W: 0.879033, X: 0.197498, Y: 0.386273, Z: 0.197708
[INFO] [1703831220.597570217] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831220.598182926] [move_group_interface]: Planning request accepted
[INFO] [1703831233.125407126] [move_group_interface]: Planning request aborted
[ERROR] [1703831233.125625584] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703831233.125680334] [RTXAgentClientNode]: move_arm  took 12528 ms
[ERROR] [1703831233.125710251] [RTXAgentClientNode]: Planning failed!
[INFO] [1703831233.125718459] [RTXAgentClientNode]: Action took 12970 ms
[INFO] [1703831233.125730001] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703831233.126982376] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831233.445052001] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831233.445208918] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831233.445218168] [RTXAgentClientNode]: Action server available after 319 ms
[ERROR] [1703831233.445236543] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831233.445241834] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831233.445247543] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831233.445252084] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831233.445259626] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703831233.445269084] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831233.445296168] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831233.445316543] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831233.445332751] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831233.445405334] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831233.451025834] [move_group_interface]: Planning request accepted
[INFO] [1703831233.529608501] [move_group_interface]: Planning request complete!
[INFO] [1703831233.530652876] [move_group_interface]: time taken to generate plan: 0.0328683 seconds
[INFO] [1703831233.530684376] [RTXAgentClientNode]: move_hand 85 ms
[INFO] [1703831233.531706251] [move_group_interface]: Execute request accepted
[INFO] [1703831233.583999501] [move_group_interface]: Execute request success!
[INFO] [1703831233.584373751] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831233.584430751] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.189211, Y: -0.147426, Z: 0.536631, RPY --  R: 0.302313, P: 0.858997, Y: 0.303164, Q -- W: 0.897970, X: 0.073173, Y: 0.427619, Z: 0.073737
[INFO] [1703831233.584442168] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.189211, Y: -0.167426, Z: 0.556631, RPY --  R: 0.302313, P: 0.858997, Y: 0.303164, Q -- W: 0.879033, X: 0.197498, Y: 0.386273, Z: 0.197708
[INFO] [1703831233.584510376] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831233.584929126] [move_group_interface]: Planning request accepted
[INFO] [1703831246.139087049] [move_group_interface]: Planning request aborted
[ERROR] [1703831246.139212340] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703831246.139265757] [RTXAgentClientNode]: move_arm  took 12554 ms
[ERROR] [1703831246.139401215] [RTXAgentClientNode]: Planning failed!
[INFO] [1703831246.139425632] [RTXAgentClientNode]: Action took 13013 ms
[INFO] [1703831246.139448465] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703831246.139878674] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831246.372908757] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831246.373005924] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831246.373103882] [RTXAgentClientNode]: Action server available after 233 ms
[ERROR] [1703831246.373124674] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831246.373130924] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831246.373137132] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831246.373142465] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831246.373146965] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831246.373151465] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831246.373164382] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831246.373169840] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831246.373248507] [move_group_interface]: MoveGroup action client/server ready
[ERROR] [1703831246.373358924] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831246.374773757] [move_group_interface]: Planning request accepted
[INFO] [1703831246.463213799] [move_group_interface]: Planning request complete!
[INFO] [1703831246.464065590] [move_group_interface]: time taken to generate plan: 0.014847 seconds
[INFO] [1703831246.464088965] [RTXAgentClientNode]: move_hand 90 ms
[INFO] [1703831246.464414340] [move_group_interface]: Execute request accepted
[INFO] [1703831246.516559674] [move_group_interface]: Execute request success!
[INFO] [1703831246.517618674] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831246.518149507] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.189211, Y: -0.147426, Z: 0.536631, RPY --  R: 0.302313, P: 0.858997, Y: 0.303164, Q -- W: 0.897970, X: 0.073173, Y: 0.427619, Z: 0.073737
[INFO] [1703831246.518304632] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.189211, Y: -0.167426, Z: 0.556631, RPY --  R: 0.302313, P: 0.858997, Y: 0.303164, Q -- W: 0.879033, X: 0.197498, Y: 0.386273, Z: 0.197708
[INFO] [1703831246.518475382] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831246.519720549] [move_group_interface]: Planning request accepted
[INFO] [1703831259.145417346] [move_group_interface]: Planning request aborted
[ERROR] [1703831259.148246263] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703831259.148283180] [RTXAgentClientNode]: move_arm  took 12630 ms
[ERROR] [1703831259.148307513] [RTXAgentClientNode]: Planning failed!
[INFO] [1703831259.148398138] [RTXAgentClientNode]: Action took 13008 ms
[INFO] [1703831259.148428763] [RTXAgentClientNode]: *** Iteration 3
[INFO] [1703831259.152540680] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831259.485265888] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831259.485473555] [RTXAgentClientNode]: Feedback: 0.000000 -0.020000 0.020000
[INFO] [1703831259.485569138] [RTXAgentClientNode]: Action server available after 337 ms
[ERROR] [1703831259.485646180] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831259.485669638] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831259.485691888] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831259.485718388] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831259.485742472] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831259.485761430] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703831259.485822055] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831259.485855180] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831259.487301222] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831259.487410388] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831259.487906222] [move_group_interface]: Planning request accepted
[INFO] [1703831259.534330722] [move_group_interface]: Planning request complete!
[INFO] [1703831259.534945347] [move_group_interface]: time taken to generate plan: 0.0197596 seconds
[INFO] [1703831259.534981847] [RTXAgentClientNode]: move_hand 49 ms
[INFO] [1703831259.535362555] [move_group_interface]: Execute request accepted
[INFO] [1703831259.588234847] [move_group_interface]: Execute request success!
[INFO] [1703831259.589019180] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 -0.020000 0.020000
[INFO] [1703831259.589142513] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.189211, Y: -0.147426, Z: 0.536631, RPY --  R: 0.302313, P: 0.858997, Y: 0.303164, Q -- W: 0.897970, X: 0.073173, Y: 0.427619, Z: 0.073737
[INFO] [1703831259.589219180] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.189211, Y: -0.167426, Z: 0.556631, RPY --  R: 0.302313, P: 0.858997, Y: 0.303164, Q -- W: 0.879033, X: 0.197498, Y: 0.386273, Z: 0.197708
[INFO] [1703831259.589398555] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831259.590519055] [move_group_interface]: Planning request accepted
^C[INFO] [1703831265.577950085] [rclcpp]: signal_handler(signum=2)
^C[INFO] [1703831300.481462171] [rclcpp]: signal_handler(signum=2)
^C[INFO] [1703831317.644670054] [rclcpp]: signal_handler(signum=2)
[ros2run]: Killed
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=10
[INFO] [1703831466.399060178] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 204.18 seconds
[INFO] [1703831466.399181762] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703831466.556080262] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703831466.574887887] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703831466.581169054] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703831467.217953262] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.32 seconds
[INFO] [1703831467.218014012] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703831467.381711637] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703831467.502542929] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703831467.503361429] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703831467.503395804] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703831467.503609512] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703831467.503901804] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703831467.513144304] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703831467.513206096] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703831467.555210637] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[INFO] [1703831467.555593637] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/attached_collision_object' for attached collision objects
[INFO] [1703831467.555616929] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703831467.555705137] [RTXAgentClientNode]: Waiting...
[INFO] [1703831467.556445471] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831467.890201554] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831467.890238346] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703831467.890265638] [RTXAgentClientNode]: Action server available after 334 ms
[ERROR] [1703831467.890282304] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831467.890287013] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831467.890291054] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831467.890294929] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831467.890299804] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831467.890304263] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703831467.890331763] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831467.891321513] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703831467.893434096] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703831467.894156554] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831467.894191721] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831467.894292804] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831467.894885471] [move_group_interface]: Planning request accepted
[INFO] [1703831467.938018763] [move_group_interface]: Planning request complete!
[INFO] [1703831467.938597763] [move_group_interface]: time taken to generate plan: 0.0128094 seconds
[INFO] [1703831467.938770346] [RTXAgentClientNode]: move_hand 48 ms
[INFO] [1703831467.939470638] [move_group_interface]: Execute request accepted
[INFO] [1703831468.104218471] [move_group_interface]: Execute request success!
[INFO] [1703831468.104490429] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703831468.104615179] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.189211, Y: -0.147426, Z: 0.536631, RPY --  R: 0.302313, P: 0.858997, Y: 0.303164, Q -- W: 0.897970, X: 0.073173, Y: 0.427619, Z: 0.073737
[INFO] [1703831468.104660179] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.189211, Y: -0.127426, Z: 0.516631, RPY --  R: 0.302313, P: 0.858997, Y: 0.303164, Q -- W: 0.879033, X: 0.197498, Y: 0.386273, Z: 0.197708
[INFO] [1703831468.104765804] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831468.107630971] [move_group_interface]: Planning request accepted
[INFO] [1703831468.255253138] [move_group_interface]: Planning request complete!
[INFO] [1703831468.256009888] [move_group_interface]: time taken to generate plan: 0.0348905 seconds
[INFO] [1703831468.256067263] [RTXAgentClientNode]: move_arm  took 151 ms
[INFO] [1703831468.256592346] [move_group_interface]: Execute request accepted
[INFO] [1703831472.505384417] [move_group_interface]: Execute request success!
[INFO] [1703831472.505542292] [RTXAgentClientNode]: Action took 4950 ms
[INFO] [1703831472.505945667] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703831472.507202500] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831472.938482459] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831472.938598876] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703831472.938665251] [RTXAgentClientNode]: Action server available after 432 ms
[ERROR] [1703831472.938763376] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831472.938836209] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831472.938888959] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831472.938941751] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831472.939009126] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831472.939062542] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703831472.938704834] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831472.939221167] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831472.939270834] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831472.939420209] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831472.942604501] [move_group_interface]: Planning request accepted
[INFO] [1703831473.002320459] [move_group_interface]: Planning request complete!
[INFO] [1703831473.002800542] [move_group_interface]: time taken to generate plan: 0.0269162 seconds
[INFO] [1703831473.005388001] [RTXAgentClientNode]: move_hand 66 ms
[INFO] [1703831473.006420001] [move_group_interface]: Execute request accepted
[INFO] [1703831473.074780667] [move_group_interface]: Execute request success!
[INFO] [1703831473.075276792] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703831473.075601876] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.172114, Y: -0.141259, Z: 0.533137, RPY --  R: 0.675057, P: 0.645607, Y: 0.676281, Q -- W: 0.879017, X: 0.196971, Y: 0.386559, Z: 0.197746
[INFO] [1703831473.075785792] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.172114, Y: -0.121259, Z: 0.513137, RPY --  R: 0.675057, P: 0.645607, Y: 0.676281, Q -- W: 0.809319, X: 0.395565, Y: 0.178196, Z: 0.395951
[INFO] [1703831473.076243584] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831473.080269792] [move_group_interface]: Planning request accepted
[INFO] [1703831473.286437376] [move_group_interface]: Planning request complete!
[INFO] [1703831473.287083501] [move_group_interface]: time taken to generate plan: 0.0203806 seconds
[INFO] [1703831473.287468459] [RTXAgentClientNode]: move_arm  took 212 ms
[INFO] [1703831473.289221709] [move_group_interface]: Execute request accepted
[INFO] [1703831479.313575462] [move_group_interface]: Execute request success!
[INFO] [1703831479.314173295] [RTXAgentClientNode]: Action took 6808 ms
[INFO] [1703831479.314227754] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703831479.314783795] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831479.603329504] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831479.603426420] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703831479.603446920] [RTXAgentClientNode]: Action server available after 289 ms
[ERROR] [1703831479.603501420] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831479.603518170] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831479.603526504] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831479.603530920] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831479.603535212] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831479.603548670] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831479.603588045] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831479.603700629] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831479.603778337] [move_group_interface]: MoveGroup action client/server ready
[ERROR] [1703831479.603507170] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831479.604271712] [move_group_interface]: Planning request accepted
[INFO] [1703831479.646710796] [move_group_interface]: Planning request complete!
[INFO] [1703831479.655412046] [move_group_interface]: time taken to generate plan: 0.0128193 seconds
[INFO] [1703831479.655446921] [RTXAgentClientNode]: move_hand 51 ms
[INFO] [1703831479.655991296] [move_group_interface]: Execute request accepted
[INFO] [1703831479.709713379] [move_group_interface]: Execute request success!
[INFO] [1703831479.709743712] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703831479.709800171] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.155185, Y: -0.142869, Z: 0.512610, RPY --  R: 0.896060, P: -0.021512, Y: 0.897838, Q -- W: 0.809929, X: 0.394453, Y: 0.179257, Z: 0.395333
[INFO] [1703831479.709819004] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.155185, Y: -0.122869, Z: 0.492610, RPY --  R: 0.896060, P: -0.021512, Y: 0.897838, Q -- W: 0.813973, X: 0.386039, Y: -0.196724, Z: 0.386938
[INFO] [1703831479.709903212] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831479.711054129] [move_group_interface]: Planning request accepted
[INFO] [1703831479.914229921] [move_group_interface]: Planning request complete!
[INFO] [1703831479.914705712] [move_group_interface]: time taken to generate plan: 0.0721295 seconds
[INFO] [1703831479.914769087] [RTXAgentClientNode]: move_arm  took 205 ms
[INFO] [1703831479.915979462] [move_group_interface]: Execute request accepted
[INFO] [1703831520.182210301] [move_group_interface]: Execute request success!
[INFO] [1703831520.182584176] [RTXAgentClientNode]: Action took 40868 ms
[INFO] [1703831520.182696467] [RTXAgentClientNode]: *** Iteration 3
[INFO] [1703831520.183698176] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831520.445436759] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831520.445476592] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703831520.445490092] [RTXAgentClientNode]: Action server available after 262 ms
[ERROR] [1703831520.445529926] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831520.445545592] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831520.445551634] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831520.445556176] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831520.445564759] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831520.445616967] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703831520.445531051] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831520.445633634] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831520.445753134] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831520.445854467] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831520.447447842] [move_group_interface]: Planning request accepted
[INFO] [1703831520.486997134] [move_group_interface]: Planning request complete!
[INFO] [1703831520.487707592] [move_group_interface]: time taken to generate plan: 0.0117168 seconds
[INFO] [1703831520.487737926] [RTXAgentClientNode]: move_hand 42 ms
[INFO] [1703831520.490576592] [move_group_interface]: Execute request accepted
[INFO] [1703831520.542855717] [move_group_interface]: Execute request success!
[INFO] [1703831520.544189967] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703831520.544245759] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.138653, Y: -0.135780, Z: 0.475438, RPY --  R: 0.657563, P: -0.665174, Y: 0.655368, Q -- W: 0.813043, X: 0.388409, Y: -0.194332, Z: 0.387730
[INFO] [1703831520.544353717] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.138653, Y: -0.115780, Z: 0.455438, RPY --  R: 0.657563, P: -0.665174, Y: 0.655368, Q -- W: 0.880902, X: 0.189504, Y: -0.390787, Z: 0.188108
[INFO] [1703831520.544522842] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831520.544927551] [move_group_interface]: Planning request accepted
[INFO] [1703831520.667893509] [move_group_interface]: Planning request complete!
[INFO] [1703831520.668300634] [move_group_interface]: time taken to generate plan: 0.0188756 seconds
[INFO] [1703831520.668709592] [RTXAgentClientNode]: move_arm  took 124 ms
[INFO] [1703831520.671912134] [move_group_interface]: Execute request accepted
[INFO] [1703831525.834661137] [move_group_interface]: Execute request success!
[INFO] [1703831525.836060053] [RTXAgentClientNode]: Action took 5653 ms
[INFO] [1703831525.836101303] [RTXAgentClientNode]: *** Iteration 4
[INFO] [1703831525.838155303] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831526.170560970] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831526.170715095] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703831526.170740220] [RTXAgentClientNode]: Action server available after 334 ms
[ERROR] [1703831526.170805928] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831526.170837553] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831526.170851553] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831526.170860887] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831526.170868637] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831526.170879887] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703831526.170902095] [RTXAgentClientNode]: VLA error: 
[INFO] [1703831526.171055303] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831526.171075428] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831526.171249887] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831526.172992720] [move_group_interface]: Planning request accepted
[INFO] [1703831526.215372470] [move_group_interface]: Planning request complete!
[INFO] [1703831526.215595720] [move_group_interface]: time taken to generate plan: 0.0139878 seconds
[INFO] [1703831526.215616095] [RTXAgentClientNode]: move_hand 44 ms
[INFO] [1703831526.215975845] [move_group_interface]: Execute request accepted
[INFO] [1703831526.293804970] [move_group_interface]: Execute request success!
[INFO] [1703831526.294212720] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703831526.294314887] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.121535, Y: -0.120977, Z: 0.434613, RPY --  R: 0.293190, P: -0.863045, Y: 0.287299, Q -- W: 0.880586, X: 0.190547, Y: -0.390512, Z: 0.189103
[INFO] [1703831526.294345762] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.121535, Y: -0.100977, Z: 0.414613, RPY --  R: 0.293190, P: -0.863045, Y: 0.287299, Q -- W: 0.898078, X: 0.072080, Y: -0.428500, Z: 0.068173
[INFO] [1703831526.294428428] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831526.294833137] [move_group_interface]: Planning request accepted
[INFO] [1703831526.418957095] [move_group_interface]: Planning request complete!
[INFO] [1703831526.419117179] [move_group_interface]: time taken to generate plan: 0.0198475 seconds
[INFO] [1703831526.419139220] [RTXAgentClientNode]: move_arm  took 124 ms
[INFO] [1703831526.420293762] [move_group_interface]: Execute request accepted
[INFO] [1703831530.720483291] [move_group_interface]: Execute request success!
[INFO] [1703831530.721355625] [RTXAgentClientNode]: Action took 4886 ms
[INFO] [1703831530.721686041] [RTXAgentClientNode]: *** Iteration 5
[INFO] [1703831530.727602041] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703831531.008367500] [RTXAgentClientNode]: Feedback callback
[INFO] [1703831531.008435166] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703831531.008464125] [RTXAgentClientNode]: Action server available after 286 ms
[ERROR] [1703831531.008520375] [RTXAgentClientNode]: move_hand began
[ERROR] [1703831531.008526500] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703831531.008533666] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703831531.008538583] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703831531.008569583] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703831531.008609833] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703831531.008615916] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703831531.008632958] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.005452, Target: 0.000000
[INFO] [1703831531.008638500] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019405, Target: 0.019500
[INFO] [1703831531.008707416] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831531.009845416] [move_group_interface]: Planning request accepted
[INFO] [1703831531.055693083] [move_group_interface]: Planning request complete!
[INFO] [1703831531.056238541] [move_group_interface]: time taken to generate plan: 0.0147536 seconds
[INFO] [1703831531.056273000] [RTXAgentClientNode]: move_hand 47 ms
[INFO] [1703831531.056947250] [move_group_interface]: Execute request accepted
[INFO] [1703831531.162205958] [move_group_interface]: Execute request success!
[INFO] [1703831531.163151416] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703831531.163269458] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.104386, Y: -0.102899, Z: 0.393390, RPY --  R: 0.114205, P: -0.893334, Y: 0.098908, Q -- W: 0.898102, X: 0.072728, Y: -0.428186, Z: 0.069134
[INFO] [1703831531.163303416] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.104386, Y: -0.082899, Z: 0.373390, RPY --  R: 0.114205, P: -0.893334, Y: 0.098908, Q -- W: 0.900540, X: 0.030091, Y: -0.433275, Z: 0.019889
[INFO] [1703831531.163399583] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703831531.165275000] [move_group_interface]: Planning request accepted
^C[INFO] [1703831541.569951171] [rclcpp]: signal_handler(signum=2)
Killed
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=10
[INFO] [1703832666.765296429] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0 seconds
[INFO] [1703832666.765344970] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703832666.830875220] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703832666.839498054] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703832666.843211429] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703832667.063346929] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0 seconds
[INFO] [1703832667.063391262] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703832667.131232679] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703832667.223523012] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703832667.224057345] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703832667.224071887] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703832667.224279429] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703832667.224483595] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703832667.229390012] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703832667.229421512] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703832667.262117720] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[INFO] [1703832667.262445887] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/attached_collision_object' for attached collision objects
[INFO] [1703832667.262470887] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703832667.262601762] [RTXAgentClientNode]: Waiting...
[INFO] [1703832667.262874429] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[WARN] [1703832667.263708304] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[ERROR] [1703832683.849137297] [RTXAgentClientNode]: Feedback callback
[INFO] [1703832683.849328297] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703832683.849452172] [RTXAgentClientNode]: Action server available after 16586 ms
[ERROR] [1703832683.849515922] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703832683.849592881] [RTXAgentClientNode]: move_hand began
[ERROR] [1703832683.849601172] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703832683.849627131] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703832683.849663756] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703832683.849746839] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703832683.849773381] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703832683.850588881] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[INFO] [1703832683.857705422] [RTXAgentClientNode]: HAND OPENNESS -- Current: -1.114286, Target: 0.000000
[INFO] [1703832683.857783297] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.000000, Target: 0.019500
[INFO] [1703832683.857887839] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832683.858369381] [move_group_interface]: Planning request accepted
[INFO] [1703832683.905984547] [move_group_interface]: Planning request complete!
[INFO] [1703832683.908603339] [move_group_interface]: time taken to generate plan: 0.0127387 seconds
[INFO] [1703832683.908705631] [RTXAgentClientNode]: move_hand 58 ms
[INFO] [1703832683.909510131] [move_group_interface]: Execute request accepted
[INFO] [1703832683.910905756] [move_group_interface]: Execute request aborted
[ERROR] [1703832683.911161381] [move_group_interface]: MoveGroupInterface::execute() failed or timeout reached
[INFO] [1703832683.911274339] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703832683.911412214] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.306938, Y: -0.000000, Z: 0.244561, RPY --  R: -0.000000, P: 0.907151, Y: -0.000000, Q -- W: 0.898886, X: -0.000000, Y: 0.438182, Z: -0.000000
[INFO] [1703832683.911454589] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.306938, Y: 0.020000, Z: 0.224561, RPY --  R: -0.000000, P: 0.907151, Y: -0.000000, Q -- W: 0.898886, X: -0.000000, Y: 0.438182, Z: -0.000000
[INFO] [1703832683.911723839] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832683.912852506] [move_group_interface]: Planning request accepted
[INFO] [1703832683.985734881] [move_group_interface]: Planning request complete!
[INFO] [1703832683.990625047] [move_group_interface]: time taken to generate plan: 0.0147237 seconds
[INFO] [1703832683.990660506] [RTXAgentClientNode]: move_arm  took 79 ms
[INFO] [1703832683.992069339] [move_group_interface]: Execute request accepted
[INFO] [1703832683.992758381] [move_group_interface]: Execute request aborted
[ERROR] [1703832683.992946339] [move_group_interface]: MoveGroupInterface::execute() failed or timeout reached
[INFO] [1703832683.993103672] [RTXAgentClientNode]: Action took 16730 ms
[INFO] [1703832683.993134422] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703832683.993634964] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[WARN] [1703832684.282254881] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: The complete state of the robot is not yet known.  Missing virtual_joint
[ERROR] [1703832684.388211339] [RTXAgentClientNode]: Feedback callback
[INFO] [1703832684.388239964] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[ERROR] [1703832684.388319089] [RTXAgentClientNode]: VLA error: 
[INFO] [1703832684.388373631] [RTXAgentClientNode]: Action server available after 395 ms
[ERROR] [1703832684.388385839] [RTXAgentClientNode]: move_hand began
[ERROR] [1703832684.388394631] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703832684.388404756] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703832684.388436964] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703832684.388488006] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703832684.388523131] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703832684.389038173] [RTXAgentClientNode]: HAND OPENNESS -- Current: -1.114286, Target: 0.000000
[INFO] [1703832684.389278089] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.000000, Target: 0.019500
[INFO] [1703832684.389408923] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832684.391392631] [move_group_interface]: Planning request accepted
[INFO] [1703832684.429442298] [move_group_interface]: Planning request complete!
[INFO] [1703832684.430818964] [move_group_interface]: time taken to generate plan: 0.0143598 seconds
[INFO] [1703832684.431114006] [RTXAgentClientNode]: move_hand 42 ms
[INFO] [1703832684.443740798] [move_group_interface]: Execute request accepted
[INFO] [1703832698.893067096] [move_group_interface]: Execute request success!
[INFO] [1703832698.893433346] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703832698.893674596] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.308721, Y: 0.000327, Z: 0.247229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: -0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703832698.893716430] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.308721, Y: 0.020327, Z: 0.227229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: 0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703832698.894335096] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832698.896752513] [move_group_interface]: Planning request accepted
[INFO] [1703832699.020079971] [move_group_interface]: Planning request complete!
[INFO] [1703832699.020656971] [move_group_interface]: time taken to generate plan: 0.0260239 seconds
[INFO] [1703832699.020699305] [RTXAgentClientNode]: move_arm  took 127 ms
[INFO] [1703832699.021183846] [move_group_interface]: Execute request accepted
[INFO] [1703832703.355819792] [move_group_interface]: Execute request success!
[INFO] [1703832703.356254126] [RTXAgentClientNode]: Action took 19362 ms
[INFO] [1703832703.356291917] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703832703.360728084] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703832703.654312959] [RTXAgentClientNode]: Feedback callback
[INFO] [1703832703.654589668] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703832703.654613334] [RTXAgentClientNode]: Action server available after 298 ms
[ERROR] [1703832703.654642084] [RTXAgentClientNode]: move_hand began
[ERROR] [1703832703.654650501] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703832703.654659543] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703832703.654665918] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703832703.654671876] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703832703.654678668] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703832703.654699626] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.542857, Target: 0.000000
[INFO] [1703832703.654712376] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.010000, Target: 0.019500
[INFO] [1703832703.654826001] [move_group_interface]: MoveGroup action client/server ready
[ERROR] [1703832703.654994376] [RTXAgentClientNode]: VLA error: 
[INFO] [1703832703.655758626] [move_group_interface]: Planning request accepted
[INFO] [1703832703.724538376] [move_group_interface]: Planning request complete!
[INFO] [1703832703.725482334] [move_group_interface]: time taken to generate plan: 0.0176228 seconds
[INFO] [1703832703.725524084] [RTXAgentClientNode]: move_hand 70 ms
[INFO] [1703832703.727035459] [move_group_interface]: Execute request accepted
[INFO] [1703832703.783658626] [move_group_interface]: Execute request success!
[INFO] [1703832703.784256793] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703832703.784317584] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291685, Y: 0.020284, Z: 0.248880, RPY --  R: 0.001271, P: 0.899455, Y: 0.000390, Q -- W: 0.900565, X: 0.000487, Y: 0.434720, Z: -0.000100
[INFO] [1703832703.784336501] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291685, Y: 0.040284, Z: 0.228880, RPY --  R: 0.001271, P: 0.899455, Y: 0.000390, Q -- W: 0.900565, X: 0.000657, Y: 0.434720, Z: 0.000452
[INFO] [1703832703.784418501] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832703.784815084] [move_group_interface]: Planning request accepted
[INFO] [1703832703.918894584] [move_group_interface]: Planning request complete!
[INFO] [1703832703.919326876] [move_group_interface]: time taken to generate plan: 0.0176328 seconds
[INFO] [1703832703.919358543] [RTXAgentClientNode]: move_arm  took 135 ms
[INFO] [1703832703.919791084] [move_group_interface]: Execute request accepted
[INFO] [1703832708.081212003] [move_group_interface]: Execute request success!
[INFO] [1703832708.082459545] [RTXAgentClientNode]: Action took 4726 ms
[INFO] [1703832708.082524420] [RTXAgentClientNode]: *** Iteration 3
[INFO] [1703832708.082904878] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703832708.374306545] [RTXAgentClientNode]: Feedback callback
[INFO] [1703832708.374475920] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703832708.374488211] [RTXAgentClientNode]: Action server available after 291 ms
[ERROR] [1703832708.374549920] [RTXAgentClientNode]: move_hand began
[ERROR] [1703832708.374570628] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703832708.374582920] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703832708.374785961] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703832708.374720836] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703832708.374880378] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703832708.374888795] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703832708.375627211] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.542857, Target: 0.000000
[INFO] [1703832708.375644378] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.010000, Target: 0.019500
[INFO] [1703832708.375813836] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832708.377434420] [move_group_interface]: Planning request accepted
[INFO] [1703832708.417824128] [move_group_interface]: Planning request complete!
[INFO] [1703832708.418086170] [move_group_interface]: time taken to generate plan: 0.0141255 seconds
[INFO] [1703832708.418131211] [RTXAgentClientNode]: move_hand 43 ms
[INFO] [1703832708.418965378] [move_group_interface]: Execute request accepted
[INFO] [1703832708.474514712] [move_group_interface]: Execute request success!
[INFO] [1703832708.475274962] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703832708.475382337] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.274670, Y: 0.040191, Z: 0.250447, RPY --  R: 0.001166, P: 0.899377, Y: 0.001179, Q -- W: 0.900582, X: 0.000269, Y: 0.434685, Z: 0.000277
[INFO] [1703832708.475480378] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.274670, Y: 0.060191, Z: 0.230447, RPY --  R: 0.001166, P: 0.899377, Y: 0.001179, Q -- W: 0.900582, X: 0.000781, Y: 0.434685, Z: 0.000784
[INFO] [1703832708.475638545] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832708.475990420] [move_group_interface]: Planning request accepted
[INFO] [1703832708.600516628] [move_group_interface]: Planning request complete!
[INFO] [1703832708.600982795] [move_group_interface]: time taken to generate plan: 0.0208486 seconds
[INFO] [1703832708.601072045] [RTXAgentClientNode]: move_arm  took 125 ms
[INFO] [1703832708.602885962] [move_group_interface]: Execute request accepted
[INFO] [1703832712.781362505] [move_group_interface]: Execute request success!
[INFO] [1703832712.782189672] [RTXAgentClientNode]: Action took 4699 ms
[INFO] [1703832712.782226755] [RTXAgentClientNode]: *** Iteration 4
[INFO] [1703832712.788959214] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703832713.181960297] [RTXAgentClientNode]: Feedback callback
[INFO] [1703832713.182027505] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703832713.182085339] [RTXAgentClientNode]: Action server available after 399 ms
[ERROR] [1703832713.182124547] [RTXAgentClientNode]: move_hand began
[ERROR] [1703832713.182157339] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703832713.182170005] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703832713.182256422] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703832713.182268464] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703832713.182283005] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703832713.182310172] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.542857, Target: 0.000000
[INFO] [1703832713.182351005] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.010000, Target: 0.019500
[INFO] [1703832713.182448255] [move_group_interface]: MoveGroup action client/server ready
[ERROR] [1703832713.182536922] [RTXAgentClientNode]: VLA error: 
[INFO] [1703832713.183277255] [move_group_interface]: Planning request accepted
[INFO] [1703832713.221155422] [move_group_interface]: Planning request complete!
[INFO] [1703832713.221883839] [move_group_interface]: time taken to generate plan: 0.0123852 seconds
[INFO] [1703832713.221939630] [RTXAgentClientNode]: move_hand 39 ms
[INFO] [1703832713.222336839] [move_group_interface]: Execute request accepted
[INFO] [1703832713.273626339] [move_group_interface]: Execute request success!
[INFO] [1703832713.273910714] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703832713.273957714] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.257516, Y: 0.060093, Z: 0.251957, RPY --  R: 0.004115, P: 0.900017, Y: 0.004526, Q -- W: 0.900441, X: 0.000869, Y: 0.434975, Z: 0.001142
[INFO] [1703832713.273985547] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.257516, Y: 0.080093, Z: 0.231957, RPY --  R: 0.004115, P: 0.900017, Y: 0.004526, Q -- W: 0.900437, X: 0.002837, Y: 0.434967, Z: 0.002932
[INFO] [1703832713.274076297] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832713.274305172] [move_group_interface]: Planning request accepted
[INFO] [1703832713.385606339] [move_group_interface]: Planning request complete!
[INFO] [1703832713.386636964] [move_group_interface]: time taken to generate plan: 0.0248065 seconds
[INFO] [1703832713.386667589] [RTXAgentClientNode]: move_arm  took 112 ms
[INFO] [1703832713.387386006] [move_group_interface]: Execute request accepted
[INFO] [1703832717.184778549] [move_group_interface]: Execute request success!
[INFO] [1703832717.185724174] [RTXAgentClientNode]: Action took 4403 ms
[INFO] [1703832717.185846007] [RTXAgentClientNode]: *** Iteration 5
[INFO] [1703832717.186582341] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703832717.463182382] [RTXAgentClientNode]: Feedback callback
[INFO] [1703832717.463214091] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[ERROR] [1703832717.463283132] [RTXAgentClientNode]: VLA error: 
[INFO] [1703832717.463301216] [RTXAgentClientNode]: Action server available after 277 ms
[ERROR] [1703832717.463308382] [RTXAgentClientNode]: move_hand began
[ERROR] [1703832717.463312799] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703832717.463318049] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703832717.463322591] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703832717.463326466] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703832717.463330591] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703832717.463350674] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.542857, Target: 0.000000
[INFO] [1703832717.463359091] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.010000, Target: 0.019500
[INFO] [1703832717.463434966] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832717.464469757] [move_group_interface]: Planning request accepted
[INFO] [1703832717.511148007] [move_group_interface]: Planning request complete!
[INFO] [1703832717.514881257] [move_group_interface]: time taken to generate plan: 0.0142276 seconds
[INFO] [1703832717.518469591] [RTXAgentClientNode]: move_hand 55 ms
[INFO] [1703832717.518889299] [move_group_interface]: Execute request accepted
[INFO] [1703832717.685320299] [move_group_interface]: Execute request success!
[INFO] [1703832717.685901258] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703832717.685976299] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.240395, Y: 0.079886, Z: 0.253569, RPY --  R: 0.010857, P: 0.900385, Y: 0.011353, Q -- W: 0.900349, X: 0.002418, Y: 0.435153, Z: 0.002749
[INFO] [1703832717.685994716] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.240395, Y: 0.099886, Z: 0.233569, RPY --  R: 0.010857, P: 0.900385, Y: 0.011353, Q -- W: 0.900322, X: 0.007358, Y: 0.435098, Z: 0.007473
[INFO] [1703832717.686074633] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832717.686323258] [move_group_interface]: Planning request accepted
[INFO] [1703832717.808948424] [move_group_interface]: Planning request complete!
[INFO] [1703832717.809371549] [move_group_interface]: time taken to generate plan: 0.0248518 seconds
[INFO] [1703832717.809499466] [RTXAgentClientNode]: move_arm  took 123 ms
[INFO] [1703832717.810783091] [move_group_interface]: Execute request accepted
[INFO] [1703832721.068017384] [move_group_interface]: Execute request success!
[INFO] [1703832721.069650801] [RTXAgentClientNode]: Action took 3883 ms
[INFO] [1703832721.070438134] [RTXAgentClientNode]: *** Iteration 6
[INFO] [1703832721.071189759] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703832721.415245843] [RTXAgentClientNode]: Feedback callback
[INFO] [1703832721.415349384] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703832721.415369676] [RTXAgentClientNode]: Action server available after 344 ms
[ERROR] [1703832721.415403051] [RTXAgentClientNode]: move_hand began
[ERROR] [1703832721.415411426] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703832721.415418218] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703832721.415423009] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703832721.415428176] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703832721.415433884] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703832721.415435176] [RTXAgentClientNode]: VLA error: 
[INFO] [1703832721.415501301] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.542857, Target: 0.000000
[INFO] [1703832721.415511134] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.010000, Target: 0.019500
[INFO] [1703832721.415589426] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832721.416453176] [move_group_interface]: Planning request accepted
[INFO] [1703832721.463895509] [move_group_interface]: Planning request complete!
[INFO] [1703832721.463969593] [move_group_interface]: time taken to generate plan: 0.0132481 seconds
[INFO] [1703832721.464031926] [RTXAgentClientNode]: move_hand 48 ms
[INFO] [1703832721.464835718] [move_group_interface]: Execute request accepted
[INFO] [1703832721.524962384] [move_group_interface]: Execute request success!
[INFO] [1703832721.525680884] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703832721.525789593] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.223432, Y: 0.099148, Z: 0.255161, RPY --  R: 0.032538, P: 0.900287, Y: 0.032839, Q -- W: 0.900260, X: 0.007503, Y: 0.435219, Z: 0.007704
[INFO] [1703832721.525807468] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.223432, Y: 0.119148, Z: 0.235161, RPY --  R: 0.032538, P: 0.900287, Y: 0.032839, Q -- W: 0.900028, X: 0.021789, Y: 0.434738, Z: 0.021858
[INFO] [1703832721.525947509] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832721.526342134] [move_group_interface]: Planning request accepted
[INFO] [1703832727.108705679] [move_group_interface]: Planning request complete!
[INFO] [1703832727.108867720] [move_group_interface]: time taken to generate plan: 5.02342 seconds
[INFO] [1703832727.108926762] [RTXAgentClientNode]: move_arm  took 5583 ms
[INFO] [1703832727.121974637] [move_group_interface]: Execute request accepted
[INFO] [1703832749.480721508] [move_group_interface]: Execute request success!
[INFO] [1703832749.483130467] [RTXAgentClientNode]: Action took 28413 ms
[INFO] [1703832749.483245842] [RTXAgentClientNode]: *** Iteration 7
[INFO] [1703832749.487696717] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703832749.681353884] [RTXAgentClientNode]: Feedback callback
[INFO] [1703832749.681572592] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703832749.681639759] [RTXAgentClientNode]: Action server available after 198 ms
[ERROR] [1703832749.681713384] [RTXAgentClientNode]: move_hand began
[ERROR] [1703832749.681755467] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703832749.681786967] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703832749.681819300] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703832749.681852550] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703832749.685153717] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703832749.685232342] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.542857, Target: 0.000000
[INFO] [1703832749.685241634] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.010000, Target: 0.019500
[INFO] [1703832749.685334759] [move_group_interface]: MoveGroup action client/server ready
[ERROR] [1703832749.681674300] [RTXAgentClientNode]: VLA error: 
[INFO] [1703832749.685963509] [move_group_interface]: Planning request accepted
[INFO] [1703832749.730697550] [move_group_interface]: Planning request complete!
[INFO] [1703832749.731358342] [move_group_interface]: time taken to generate plan: 0.0124719 seconds
[INFO] [1703832749.731388467] [RTXAgentClientNode]: move_hand 46 ms
[INFO] [1703832749.732792759] [move_group_interface]: Execute request accepted
[INFO] [1703832749.785737259] [move_group_interface]: Execute request success!
[INFO] [1703832749.785999967] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703832749.786058467] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.346434, Y: -0.237032, Z: 0.143257, RPY --  R: -2.099298, P: 0.300514, Y: -0.228476, Q -- W: 0.503855, X: -0.843392, Y: 0.171786, Z: 0.072861
[INFO] [1703832749.786086759] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.346434, Y: -0.217032, Z: 0.123257, RPY --  R: -2.099298, P: 0.300514, Y: -0.228476, Q -- W: 0.474259, X: -0.860383, Y: -0.023701, Z: -0.185087
[INFO] [1703832749.786166425] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832749.788298342] [move_group_interface]: Planning request accepted
[INFO] [1703832749.947173759] [move_group_interface]: Planning request complete!
[INFO] [1703832749.947392634] [move_group_interface]: time taken to generate plan: 0.055159 seconds
[INFO] [1703832749.947452175] [RTXAgentClientNode]: move_arm  took 161 ms
[INFO] [1703832749.948611759] [move_group_interface]: Execute request accepted
[INFO] [1703832779.753063842] [move_group_interface]: Execute request success!
[INFO] [1703832779.753806884] [RTXAgentClientNode]: Action took 30269 ms
[INFO] [1703832779.753847467] [RTXAgentClientNode]: *** Iteration 8
[INFO] [1703832779.755295842] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703832780.083301259] [RTXAgentClientNode]: Feedback callback
[INFO] [1703832780.083326384] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703832780.083337592] [RTXAgentClientNode]: Action server available after 329 ms
[ERROR] [1703832780.083372259] [RTXAgentClientNode]: move_hand began
[ERROR] [1703832780.083383509] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703832780.083394842] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703832780.083402634] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703832780.083438050] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703832780.083447425] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703832780.083455717] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703832780.083477467] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.542857, Target: 0.000000
[INFO] [1703832780.083487509] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.010000, Target: 0.019500
[INFO] [1703832780.083571509] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832780.084192342] [move_group_interface]: Planning request accepted
[INFO] [1703832780.123835217] [move_group_interface]: Planning request complete!
[INFO] [1703832780.124885009] [move_group_interface]: time taken to generate plan: 0.00652521 seconds
[INFO] [1703832780.124904800] [RTXAgentClientNode]: move_hand 41 ms
[INFO] [1703832780.125643717] [move_group_interface]: Execute request accepted
[INFO] [1703832780.226998842] [move_group_interface]: Execute request success!
[INFO] [1703832780.227650175] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703832780.227707342] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.320900, Y: -0.213209, Z: 0.114000, RPY --  R: -2.112538, P: -0.348065, Y: -0.140779, Q -- W: -0.472888, X: 0.861246, Y: 0.024702, Z: 0.184452
[INFO] [1703832780.227759467] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.320900, Y: -0.193209, Z: 0.094000, RPY --  R: -2.112538, P: -0.348065, Y: -0.140779, Q -- W: 0.494091, X: -0.849259, Y: -0.145303, Z: 0.116274
[INFO] [1703832780.227840175] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832780.229761175] [move_group_interface]: Planning request accepted
[INFO] [1703832780.352092217] [move_group_interface]: Planning request complete!
[INFO] [1703832780.353045717] [move_group_interface]: time taken to generate plan: 0.033542 seconds
[INFO] [1703832780.353093801] [RTXAgentClientNode]: move_arm  took 125 ms
[INFO] [1703832780.353639884] [move_group_interface]: Execute request accepted
[INFO] [1703832786.129816845] [move_group_interface]: Execute request success!
[INFO] [1703832786.130735553] [RTXAgentClientNode]: Action took 6376 ms
[INFO] [1703832786.130772262] [RTXAgentClientNode]: *** Iteration 9
[INFO] [1703832786.131173762] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703832786.488534345] [RTXAgentClientNode]: Feedback callback
[INFO] [1703832786.488600178] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703832786.488622720] [RTXAgentClientNode]: Action server available after 357 ms
[ERROR] [1703832786.488645845] [RTXAgentClientNode]: move_hand began
[ERROR] [1703832786.488653012] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703832786.488703345] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703832786.488747512] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703832786.488752553] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703832786.488756428] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703832786.488760887] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703832786.488778012] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.542857, Target: 0.000000
[INFO] [1703832786.488784053] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.010000, Target: 0.019500
[INFO] [1703832786.488860762] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832786.490368887] [move_group_interface]: Planning request accepted
[INFO] [1703832786.542823428] [move_group_interface]: Planning request complete!
[INFO] [1703832786.543236470] [move_group_interface]: time taken to generate plan: 0.0132973 seconds
[INFO] [1703832786.543262470] [RTXAgentClientNode]: move_hand 54 ms
[INFO] [1703832786.543731053] [move_group_interface]: Execute request accepted
[INFO] [1703832786.596293803] [move_group_interface]: Execute request success!
[INFO] [1703832786.596983803] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703832786.597044512] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.295422, Y: -0.203084, Z: 0.095628, RPY --  R: -2.076885, P: 0.051990, Y: 0.368550, Q -- W: -0.494697, X: 0.849158, Y: 0.144854, Z: -0.114987
[INFO] [1703832786.597056887] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.295422, Y: -0.183084, Z: 0.075628, RPY --  R: -2.076885, P: 0.051990, Y: 0.368550, Q -- W: 0.502904, X: -0.844324, Y: 0.170793, Z: 0.070955
[INFO] [1703832786.597138262] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703832786.597986637] [move_group_interface]: Planning request accepted
[INFO] [1703832786.702672554] [move_group_interface]: Planning request complete!
[INFO] [1703832786.703218845] [move_group_interface]: time taken to generate plan: 0.0190328 seconds
[INFO] [1703832786.703303970] [RTXAgentClientNode]: move_arm  took 106 ms
[INFO] [1703832786.704023262] [move_group_interface]: Execute request accepted
[INFO] [1703832792.375965084] [move_group_interface]: Execute request success!
[INFO] [1703832792.376122834] [RTXAgentClientNode]: Action took 6244 ms
[INFO] [1703832792.378000667] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[INFO] [1703832792.378299084] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
double free or corruption (fasttop)
[ros2run]: Aborted
root@d3ec6893a1b6:/simply_ws# ^C
root@d3ec6893a1b6:/simply_ws# ^C
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=10
[INFO] [1703833201.415033884] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.8 seconds
[INFO] [1703833201.415099634] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703833201.616429759] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703833201.657340259] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703833201.668616176] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703833202.055345135] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.14 seconds
[INFO] [1703833202.055373426] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703833202.199316968] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703833202.377243343] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703833202.378010218] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703833202.378029635] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703833202.378436801] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703833202.379421176] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703833202.396784718] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703833202.396837885] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703833202.437829176] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[INFO] [1703833202.440046926] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/attached_collision_object' for attached collision objects
[INFO] [1703833202.440191968] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703833202.441282551] [RTXAgentClientNode]: Waiting...
[INFO] [1703833212.753397625] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833213.087708584] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833213.088004501] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833213.088031959] [RTXAgentClientNode]: Action server available after 10648 ms
[ERROR] [1703833213.088060876] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833213.088066751] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833213.088072626] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833213.088077001] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833213.088081251] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833213.088092626] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703833213.088205209] [RTXAgentClientNode]: VLA error: 
[INFO] [1703833213.089867251] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703833213.091203084] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703833213.116321001] [RTXAgentClientNode]: HAND OPENNESS -- Current: -1.114286, Target: 1.000000
[INFO] [1703833213.116361459] [RTXAgentClientNode]: FINGER JOINT -- Current: -0.000000, Target: 0.019500
[INFO] [1703833213.116500876] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833213.117147042] [move_group_interface]: Planning request accepted
[INFO] [1703833213.181694376] [move_group_interface]: Planning request complete!
[INFO] [1703833213.182375209] [move_group_interface]: time taken to generate plan: 0.0259467 seconds
[INFO] [1703833213.182576917] [RTXAgentClientNode]: move_hand 94 ms
[INFO] [1703833213.183982626] [move_group_interface]: Execute request accepted
[INFO] [1703833226.326136799] [move_group_interface]: Execute request success!
[INFO] [1703833226.327063424] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833226.327156590] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.308721, Y: 0.000327, Z: 0.247229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: -0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703833226.327178049] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.308721, Y: 0.020327, Z: 0.227229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: 0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703833226.327277007] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833226.327990174] [move_group_interface]: Planning request accepted
[INFO] [1703833226.440314965] [move_group_interface]: Planning request complete!
[INFO] [1703833226.440434715] [move_group_interface]: time taken to generate plan: 0.0298649 seconds
[INFO] [1703833226.440586424] [RTXAgentClientNode]: move_arm  took 113 ms
[INFO] [1703833226.442009590] [move_group_interface]: Execute request accepted
[INFO] [1703833230.105042717] [move_group_interface]: Execute request success!
[INFO] [1703833230.105313092] [RTXAgentClientNode]: Action took 27665 ms
[INFO] [1703833230.105349592] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703833230.108403425] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833230.519181259] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833230.519285717] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833230.519887342] [RTXAgentClientNode]: Action server available after 414 ms
[ERROR] [1703833230.520023301] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833230.520076717] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833230.520118717] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833230.520171592] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833230.520211009] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833230.520262634] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703833230.520290967] [RTXAgentClientNode]: VLA error: 
[INFO] [1703833230.520405426] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.542857, Target: 1.000000
[INFO] [1703833230.520464467] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.010000, Target: 0.027500
[INFO] [1703833230.520599842] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833230.521653926] [move_group_interface]: Planning request accepted
[INFO] [1703833230.552740176] [move_group_interface]: Planning request complete!
[INFO] [1703833230.552754009] [move_group_interface]: time taken to generate plan: 0.00248204 seconds
[INFO] [1703833230.552908926] [RTXAgentClientNode]: move_hand 32 ms
[INFO] [1703833230.553989717] [move_group_interface]: Execute request accepted
[INFO] [1703833230.661727092] [move_group_interface]: Execute request success!
[INFO] [1703833230.661821926] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833230.661890759] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291743, Y: 0.020229, Z: 0.248829, RPY --  R: 0.001229, P: 0.900754, Y: 0.001035, Q -- W: 0.900283, X: 0.000328, Y: 0.435305, Z: 0.000199
[INFO] [1703833230.661906301] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291743, Y: 0.040229, Z: 0.228829, RPY --  R: 0.001229, P: 0.900754, Y: 0.001035, Q -- W: 0.900283, X: 0.000778, Y: 0.435304, Z: 0.000733
[INFO] [1703833230.661990301] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833230.663480467] [move_group_interface]: Planning request accepted
[INFO] [1703833230.785450426] [move_group_interface]: Planning request complete!
[INFO] [1703833230.785864426] [move_group_interface]: time taken to generate plan: 0.0243784 seconds
[INFO] [1703833230.785935384] [RTXAgentClientNode]: move_arm  took 124 ms
[INFO] [1703833230.786576717] [move_group_interface]: Execute request accepted
[INFO] [1703833234.566893094] [move_group_interface]: Execute request success!
[INFO] [1703833234.569314053] [RTXAgentClientNode]: Action took 4463 ms
[INFO] [1703833234.569618928] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703833234.570346719] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833234.888590844] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833234.888696969] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833234.888730261] [RTXAgentClientNode]: Action server available after 319 ms
[ERROR] [1703833234.888754511] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833234.888760178] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833234.888765594] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833234.888770344] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833234.888773928] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833234.888778094] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833234.888794136] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.542857, Target: 1.000000
[INFO] [1703833234.888800011] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.010000, Target: 0.027500
[ERROR] [1703833234.888870761] [RTXAgentClientNode]: VLA error: 
[INFO] [1703833234.888891636] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833234.889651428] [move_group_interface]: Planning request accepted
[INFO] [1703833234.931373803] [move_group_interface]: Planning request complete!
[INFO] [1703833234.934984803] [move_group_interface]: time taken to generate plan: 0.0128923 seconds
[INFO] [1703833234.935141011] [RTXAgentClientNode]: move_hand 46 ms
[INFO] [1703833234.940460178] [move_group_interface]: Execute request accepted
[INFO] [1703833234.988543053] [move_group_interface]: Execute request success!
[INFO] [1703833234.988778219] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833234.988832428] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.274736, Y: 0.039989, Z: 0.250455, RPY --  R: 0.004118, P: 0.901838, Y: 0.004579, Q -- W: 0.900045, X: 0.000856, Y: 0.435795, Z: 0.001163
[INFO] [1703833234.988844803] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.274736, Y: 0.059989, Z: 0.230455, RPY --  R: 0.004118, P: 0.901838, Y: 0.004579, Q -- W: 0.900041, X: 0.002851, Y: 0.435787, Z: 0.002958
[INFO] [1703833234.988918886] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833234.990569136] [move_group_interface]: Planning request accepted
[INFO] [1703833235.096488178] [move_group_interface]: Planning request complete!
[INFO] [1703833235.097220594] [move_group_interface]: time taken to generate plan: 0.0167355 seconds
[INFO] [1703833235.097299303] [RTXAgentClientNode]: move_arm  took 108 ms
[INFO] [1703833235.098091303] [move_group_interface]: Execute request accepted
[INFO] [1703833238.778459055] [move_group_interface]: Execute request success!
[INFO] [1703833238.779128596] [RTXAgentClientNode]: Action took 4209 ms
[INFO] [1703833238.779204763] [RTXAgentClientNode]: *** Iteration 3
[INFO] [1703833238.779870471] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833239.100140555] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833239.100177513] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833239.100188221] [RTXAgentClientNode]: Action server available after 320 ms
[ERROR] [1703833239.100206180] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833239.100212013] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833239.100213763] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703833239.100244430] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833239.100261721] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833239.100299513] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833239.100311180] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833239.100333055] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.542857, Target: 1.000000
[INFO] [1703833239.100342346] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.010000, Target: 0.027500
[INFO] [1703833239.100461221] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833239.101881263] [move_group_interface]: Planning request accepted
[INFO] [1703833239.178143013] [move_group_interface]: Planning request complete!
[INFO] [1703833239.178320846] [move_group_interface]: time taken to generate plan: 0.0222075 seconds
[INFO] [1703833239.178343471] [RTXAgentClientNode]: move_hand 78 ms
[INFO] [1703833239.178636430] [move_group_interface]: Execute request accepted
[INFO] [1703833239.231194263] [move_group_interface]: Execute request success!
[INFO] [1703833239.232647638] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833239.232726263] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.257832, Y: 0.059693, Z: 0.252038, RPY --  R: 0.012331, P: 0.902865, Y: 0.011506, Q -- W: 0.899807, X: 0.003038, Y: 0.436271, Z: 0.002487
[INFO] [1703833239.232758263] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.257832, Y: 0.079693, Z: 0.232038, RPY --  R: 0.012331, P: 0.902865, Y: 0.011506, Q -- W: 0.899776, X: 0.008057, Y: 0.436207, Z: 0.007866
[INFO] [1703833239.232874763] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833239.233518305] [move_group_interface]: Planning request accepted
[INFO] [1703833239.333157180] [move_group_interface]: Planning request complete!
[INFO] [1703833239.333392888] [move_group_interface]: time taken to generate plan: 0.0159557 seconds
[INFO] [1703833239.333428430] [RTXAgentClientNode]: move_arm  took 100 ms
[INFO] [1703833239.333943471] [move_group_interface]: Execute request accepted
[INFO] [1703833242.608672250] [move_group_interface]: Execute request success!
[INFO] [1703833242.609280125] [RTXAgentClientNode]: Action took 3828 ms
[INFO] [1703833242.609368917] [RTXAgentClientNode]: *** Iteration 4
[INFO] [1703833242.610263584] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833242.977176709] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833242.977308542] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833242.977344834] [RTXAgentClientNode]: Action server available after 367 ms
[ERROR] [1703833242.977369292] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833242.977374917] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833242.977395709] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833242.977401001] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833242.977416126] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833242.977457459] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703833242.977426417] [RTXAgentClientNode]: VLA error: 
[INFO] [1703833242.977478292] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.542857, Target: 1.000000
[INFO] [1703833242.977601334] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.010000, Target: 0.027500
[INFO] [1703833242.977707917] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833242.979287959] [move_group_interface]: Planning request accepted
[INFO] [1703833243.032996459] [move_group_interface]: Planning request complete!
[INFO] [1703833243.033654376] [move_group_interface]: time taken to generate plan: 0.0227968 seconds
[INFO] [1703833243.033907126] [RTXAgentClientNode]: move_hand 56 ms
[INFO] [1703833243.034980917] [move_group_interface]: Execute request accepted
[INFO] [1703833243.086347292] [move_group_interface]: Execute request success!
[INFO] [1703833243.087047042] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833243.087100417] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.240819, Y: 0.078987, Z: 0.253616, RPY --  R: 0.033546, P: 0.903261, Y: 0.033617, Q -- W: 0.899606, X: 0.007754, Y: 0.436564, Z: 0.007801
[INFO] [1703833243.087161626] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.240819, Y: 0.098987, Z: 0.233616, RPY --  R: 0.033546, P: 0.903261, Y: 0.033617, Q -- W: 0.899360, X: 0.022423, Y: 0.436056, Z: 0.022439
[INFO] [1703833243.087243251] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833243.088194042] [move_group_interface]: Planning request accepted
[INFO] [1703833248.598181920] [move_group_interface]: Planning request complete!
[INFO] [1703833248.598402670] [move_group_interface]: time taken to generate plan: 5.02942 seconds
[INFO] [1703833248.598482712] [RTXAgentClientNode]: move_arm  took 5511 ms
[INFO] [1703833248.611658087] [move_group_interface]: Execute request accepted
^C[INFO] [1703833266.664879345] [rclcpp]: signal_handler(signum=2)
^[[AKilled
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=10
[INFO] [1703833477.646117554] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 9.34 seconds
[INFO] [1703833477.646172471] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703833477.827035679] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703833477.841125637] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703833477.846275637] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703833477.986888762] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.06 seconds
[INFO] [1703833477.986966804] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703833478.098479888] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703833478.215239679] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703833478.215881638] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703833478.215893804] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703833478.216151679] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703833478.216681846] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703833478.240353763] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703833478.240389013] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703833478.273369138] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703833478.273608429] [RTXAgentClientNode]: Waiting...
[INFO] [1703833478.275406846] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833478.631661679] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833478.631835596] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833478.633383179] [RTXAgentClientNode]: Action server available after 359 ms
[ERROR] [1703833478.633416429] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833478.633422429] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833478.633434471] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833478.633440763] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833478.633445096] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833478.633448679] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703833478.633449596] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833478.634979513] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703833478.642061138] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703833478.666948888] [RTXAgentClientNode]: HAND OPENNESS -- Current: -1.114286, Target: 1.000000
[INFO] [1703833478.666988554] [RTXAgentClientNode]: FINGER JOINT -- Current: -0.000000, Target: 0.019500
[INFO] [1703833478.667066013] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833478.667710221] [move_group_interface]: Planning request accepted
[INFO] [1703833478.720144554] [move_group_interface]: Planning request complete!
[INFO] [1703833478.720921971] [move_group_interface]: time taken to generate plan: 0.0173484 seconds
[INFO] [1703833478.720969846] [RTXAgentClientNode]: move_hand 87 ms
[INFO] [1703833478.722094096] [move_group_interface]: Execute request accepted
[INFO] [1703833496.030177507] [move_group_interface]: Execute request success!
[INFO] [1703833496.031164340] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833496.031477798] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.308721, Y: 0.000327, Z: 0.247229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: -0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703833496.031524048] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.308721, Y: 0.020327, Z: 0.227229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: 0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703833496.031726715] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833496.033038340] [move_group_interface]: Planning request accepted
[INFO] [1703833496.158260673] [move_group_interface]: Planning request complete!
[INFO] [1703833496.158727423] [move_group_interface]: time taken to generate plan: 0.0166234 seconds
[INFO] [1703833496.158771507] [RTXAgentClientNode]: move_arm  took 127 ms
[INFO] [1703833496.160043340] [move_group_interface]: Execute request accepted
[INFO] [1703833499.664375800] [move_group_interface]: Execute request success!
[INFO] [1703833499.664546717] [RTXAgentClientNode]: Action took 21390 ms
[INFO] [1703833500.164985967] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703833500.165545717] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833500.597237342] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833500.597275634] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833500.597304134] [RTXAgentClientNode]: Action server available after 432 ms
[ERROR] [1703833500.597322842] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833500.597330217] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833500.597335092] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833500.597339301] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833500.597343467] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833500.597348217] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833500.597370676] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.001667, Target: 1.000000
[INFO] [1703833500.597377009] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019471, Target: 0.037000
[ERROR] [1703833500.597408592] [RTXAgentClientNode]: VLA error: 
[INFO] [1703833500.597514176] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833500.598745967] [move_group_interface]: Planning request accepted
[INFO] [1703833500.652936926] [move_group_interface]: Planning request complete!
[INFO] [1703833500.653255801] [move_group_interface]: time taken to generate plan: 0.0217394 seconds
[INFO] [1703833500.653295801] [RTXAgentClientNode]: move_hand 55 ms
[INFO] [1703833500.653710384] [move_group_interface]: Execute request accepted
[INFO] [1703833516.848326502] [move_group_interface]: Execute request success!
[INFO] [1703833516.849291461] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833516.849428586] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291605, Y: 0.020313, Z: 0.248935, RPY --  R: 0.001648, P: 0.900849, Y: 0.002444, Q -- W: 0.900262, X: 0.000210, Y: 0.435348, Z: 0.000741
[INFO] [1703833516.849455169] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291605, Y: 0.040313, Z: 0.228935, RPY --  R: 0.001648, P: 0.900849, Y: 0.002444, Q -- W: 0.900261, X: 0.001274, Y: 0.435346, Z: 0.001459
[INFO] [1703833516.849539252] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833516.851345169] [move_group_interface]: Planning request accepted
[INFO] [1703833516.965017336] [move_group_interface]: Planning request complete!
[INFO] [1703833516.966261586] [move_group_interface]: time taken to generate plan: 0.0219598 seconds
[INFO] [1703833516.966306752] [RTXAgentClientNode]: move_arm  took 117 ms
[INFO] [1703833516.968400836] [move_group_interface]: Execute request accepted
[INFO] [1703833520.603127254] [move_group_interface]: Execute request success!
[INFO] [1703833520.603374629] [RTXAgentClientNode]: Action took 20438 ms
[INFO] [1703833521.107245254] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703833521.108161088] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833521.440409088] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833521.440502963] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833521.440624755] [RTXAgentClientNode]: Action server available after 333 ms
[ERROR] [1703833521.440676046] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703833521.440762880] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833521.441523213] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833521.441601963] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833521.441648255] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833521.441666421] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833521.441693588] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833521.441736796] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996062, Target: 1.000000
[INFO] [1703833521.441770255] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036931, Target: 0.037000
[INFO] [1703833521.441892755] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833521.443542255] [move_group_interface]: Planning request accepted
[INFO] [1703833521.500129588] [move_group_interface]: Planning request complete!
[INFO] [1703833521.501429921] [move_group_interface]: time taken to generate plan: 0.0230408 seconds
[INFO] [1703833521.501477546] [RTXAgentClientNode]: move_hand 59 ms
[INFO] [1703833521.501986213] [move_group_interface]: Execute request accepted
[INFO] [1703833521.603944171] [move_group_interface]: Execute request success!
[INFO] [1703833521.604146671] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833521.604277046] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.274488, Y: 0.040298, Z: 0.250504, RPY --  R: 0.005006, P: 0.900906, Y: 0.004845, Q -- W: 0.900247, X: 0.001198, Y: 0.435376, Z: 0.001091
[INFO] [1703833521.604357130] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.274488, Y: 0.060298, Z: 0.230504, RPY --  R: 0.005006, P: 0.900906, Y: 0.004845, Q -- W: 0.900242, X: 0.003308, Y: 0.435365, Z: 0.003271
[INFO] [1703833521.604490046] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833521.604892755] [move_group_interface]: Planning request accepted
[INFO] [1703833521.758590921] [move_group_interface]: Planning request complete!
[INFO] [1703833521.759302088] [move_group_interface]: time taken to generate plan: 0.0291358 seconds
[INFO] [1703833521.759337838] [RTXAgentClientNode]: move_arm  took 155 ms
[INFO] [1703833521.761431588] [move_group_interface]: Execute request accepted
[INFO] [1703833525.344965173] [move_group_interface]: Execute request success!
[INFO] [1703833525.346604048] [RTXAgentClientNode]: Action took 4239 ms
[INFO] [1703833525.846981090] [RTXAgentClientNode]: *** Iteration 3
[INFO] [1703833525.851635382] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833526.157922548] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833526.157957965] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833526.157969548] [RTXAgentClientNode]: Action server available after 310 ms
[ERROR] [1703833526.158001548] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833526.158007257] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833526.158013548] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833526.158018923] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833526.158022882] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833526.158028507] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833526.158043673] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996062, Target: 1.000000
[INFO] [1703833526.158066298] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036931, Target: 0.037000
[INFO] [1703833526.158192257] [move_group_interface]: MoveGroup action client/server ready
[ERROR] [1703833526.158203382] [RTXAgentClientNode]: VLA error: 
[INFO] [1703833526.158888548] [move_group_interface]: Planning request accepted
[INFO] [1703833526.212292299] [move_group_interface]: Planning request complete!
[INFO] [1703833526.213271132] [move_group_interface]: time taken to generate plan: 0.0238411 seconds
[INFO] [1703833526.213582257] [RTXAgentClientNode]: move_hand 55 ms
[INFO] [1703833526.214774174] [move_group_interface]: Execute request accepted
[INFO] [1703833526.269244840] [move_group_interface]: Execute request success!
[INFO] [1703833526.269650465] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833526.269726299] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.257460, Y: 0.060066, Z: 0.252086, RPY --  R: 0.013624, P: 0.901865, Y: 0.012751, Q -- W: 0.900021, X: 0.003352, Y: 0.435825, Z: 0.002770
[INFO] [1703833526.269744382] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.257460, Y: 0.080066, Z: 0.232086, RPY --  R: 0.013624, P: 0.901865, Y: 0.012751, Q -- W: 0.899983, X: 0.008909, Y: 0.435747, Z: 0.008707
[INFO] [1703833526.269816632] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833526.270364174] [move_group_interface]: Planning request accepted
[INFO] [1703833526.407127382] [move_group_interface]: Planning request complete!
[INFO] [1703833526.407422090] [move_group_interface]: time taken to generate plan: 0.0286036 seconds
[INFO] [1703833526.407448257] [RTXAgentClientNode]: move_arm  took 137 ms
[INFO] [1703833526.407774257] [move_group_interface]: Execute request accepted
[INFO] [1703833529.673291717] [move_group_interface]: Execute request success!
[INFO] [1703833529.673573133] [RTXAgentClientNode]: Action took 3826 ms
[INFO] [1703833530.174318717] [RTXAgentClientNode]: *** Iteration 4
[INFO] [1703833530.174988009] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833530.513436842] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833530.513566801] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833530.513608384] [RTXAgentClientNode]: Action server available after 338 ms
[ERROR] [1703833530.513632301] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833530.513643801] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833530.513646009] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703833530.513664217] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833530.513677176] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833530.513686842] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833530.513701759] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833530.513724717] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996062, Target: 1.000000
[INFO] [1703833530.513736051] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036931, Target: 0.037000
[INFO] [1703833530.513860967] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833530.514323259] [move_group_interface]: Planning request accepted
[INFO] [1703833530.550840092] [move_group_interface]: Planning request complete!
[INFO] [1703833530.551813801] [move_group_interface]: time taken to generate plan: 0.00638517 seconds
[INFO] [1703833530.554169884] [RTXAgentClientNode]: move_hand 40 ms
[INFO] [1703833530.554771134] [move_group_interface]: Execute request accepted
[INFO] [1703833530.606088801] [move_group_interface]: Execute request success!
[INFO] [1703833530.606452092] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833530.606570217] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.240456, Y: 0.079329, Z: 0.253749, RPY --  R: 0.038156, P: 0.902368, Y: 0.038782, Q -- W: 0.899760, X: 0.008711, Y: 0.436203, Z: 0.009130
[INFO] [1703833530.606591009] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.240456, Y: 0.099329, Z: 0.233749, RPY --  R: 0.038156, P: 0.902368, Y: 0.038782, Q -- W: 0.899437, X: 0.025618, Y: 0.435537, Z: 0.025763
[INFO] [1703833530.606778176] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833530.607149259] [move_group_interface]: Planning request accepted
[INFO] [1703833536.114400595] [move_group_interface]: Planning request complete!
[INFO] [1703833536.115119845] [move_group_interface]: time taken to generate plan: 5.02657 seconds
[INFO] [1703833536.115210637] [RTXAgentClientNode]: move_arm  took 5508 ms
[INFO] [1703833536.116555470] [move_group_interface]: Execute request accepted
[INFO] [1703833556.935245591] [move_group_interface]: Execute request success!
[INFO] [1703833556.936284007] [RTXAgentClientNode]: Action took 26761 ms
[INFO] [1703833557.436409216] [RTXAgentClientNode]: *** Iteration 5
[INFO] [1703833557.437063757] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833557.718428716] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833557.718540466] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833557.718592924] [RTXAgentClientNode]: Action server available after 282 ms
[ERROR] [1703833557.718648633] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703833557.718721424] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833557.718738133] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833557.718754424] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833557.718766258] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833557.718774966] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833557.718786841] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833557.718806758] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996062, Target: 1.000000
[INFO] [1703833557.718820549] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036931, Target: 0.037000
[INFO] [1703833557.718910716] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833557.719759424] [move_group_interface]: Planning request accepted
[INFO] [1703833557.787326008] [move_group_interface]: Planning request complete!
[INFO] [1703833557.787551383] [move_group_interface]: time taken to generate plan: 0.0130442 seconds
[INFO] [1703833557.787608674] [RTXAgentClientNode]: move_hand 68 ms
[INFO] [1703833557.788314508] [move_group_interface]: Execute request accepted
[INFO] [1703833557.845743674] [move_group_interface]: Execute request success!
[INFO] [1703833557.847478341] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833557.847540174] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.074875, Y: -0.298772, Z: 0.076748, RPY --  R: -1.990595, P: 0.636557, Y: -1.042481, Q -- W: 0.578999, X: -0.606166, Y: 0.544462, Z: -0.029740
[INFO] [1703833557.847553049] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.074875, Y: -0.278772, Z: 0.056748, RPY --  R: -1.990595, P: 0.636557, Y: -1.042481, Q -- W: 0.317547, X: -0.775784, Y: -0.249068, Z: -0.485065
[INFO] [1703833557.847618091] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833557.848285424] [move_group_interface]: Planning request accepted
[INFO] [1703833570.434881930] [move_group_interface]: Planning request aborted
[ERROR] [1703833570.435880180] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703833570.436072597] [RTXAgentClientNode]: move_arm  took 12588 ms
[ERROR] [1703833570.436092430] [RTXAgentClientNode]: Planning failed!
[INFO] [1703833570.436103180] [RTXAgentClientNode]: Action took 12999 ms
[INFO] [1703833570.935632666] [RTXAgentClientNode]: *** Iteration 6
[INFO] [1703833570.937184958] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833571.244748833] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833571.244802333] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[ERROR] [1703833571.244942375] [RTXAgentClientNode]: VLA error: 
[INFO] [1703833571.245214666] [RTXAgentClientNode]: Action server available after 309 ms
[ERROR] [1703833571.245633416] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833571.245660666] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833571.245677083] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833571.245683416] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833571.245687458] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833571.245692125] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833571.245707708] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996062, Target: 1.000000
[INFO] [1703833571.245712583] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036931, Target: 0.037000
[INFO] [1703833571.247821208] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833571.249485791] [move_group_interface]: Planning request accepted
[INFO] [1703833571.291859916] [move_group_interface]: Planning request complete!
[INFO] [1703833571.292735750] [move_group_interface]: time taken to generate plan: 0.0138199 seconds
[INFO] [1703833571.292835250] [RTXAgentClientNode]: move_hand 47 ms
[INFO] [1703833571.295560000] [move_group_interface]: Execute request accepted
[INFO] [1703833571.346467916] [move_group_interface]: Execute request success!
[INFO] [1703833571.347330333] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833571.347450166] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.074708, Y: -0.298708, Z: 0.076733, RPY --  R: -1.990765, P: 0.636373, Y: -1.043110, Q -- W: 0.578915, X: -0.606088, Y: 0.544626, Z: -0.029947
[INFO] [1703833571.347503750] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.074708, Y: -0.278708, Z: 0.056733, RPY --  R: -1.990765, P: 0.636373, Y: -1.043110, Q -- W: 0.317378, X: -0.775730, Y: -0.249406, Z: -0.485088
[INFO] [1703833571.347599375] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833571.351015875] [move_group_interface]: Planning request accepted
[INFO] [1703833583.922430464] [move_group_interface]: Planning request aborted
[ERROR] [1703833583.922611672] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703833583.923041131] [RTXAgentClientNode]: move_arm  took 12575 ms
[ERROR] [1703833583.923066756] [RTXAgentClientNode]: Planning failed!
[INFO] [1703833583.923082506] [RTXAgentClientNode]: Action took 12986 ms
[INFO] [1703833584.423226464] [RTXAgentClientNode]: *** Iteration 7
[INFO] [1703833584.433186881] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833584.646891756] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833584.646941048] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[ERROR] [1703833584.646991714] [RTXAgentClientNode]: VLA error: 
[INFO] [1703833584.647034464] [RTXAgentClientNode]: Action server available after 223 ms
[ERROR] [1703833584.647045089] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833584.647058464] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833584.647067214] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833584.647075548] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833584.647083131] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833584.647091631] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833584.647111756] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996062, Target: 1.000000
[INFO] [1703833584.647120548] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036931, Target: 0.037000
[INFO] [1703833584.647185048] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833584.648428548] [move_group_interface]: Planning request accepted
[INFO] [1703833584.691914173] [move_group_interface]: Planning request complete!
[INFO] [1703833584.692873173] [move_group_interface]: time taken to generate plan: 0.0131996 seconds
[INFO] [1703833584.693117173] [RTXAgentClientNode]: move_hand 46 ms
[INFO] [1703833584.694292673] [move_group_interface]: Execute request accepted
[INFO] [1703833584.756967298] [move_group_interface]: Execute request success!
[INFO] [1703833584.757139339] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833584.757194173] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.074708, Y: -0.298708, Z: 0.076733, RPY --  R: -1.990765, P: 0.636373, Y: -1.043110, Q -- W: 0.578915, X: -0.606088, Y: 0.544626, Z: -0.029947
[INFO] [1703833584.757207839] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.074708, Y: -0.278708, Z: 0.056733, RPY --  R: -1.990765, P: 0.636373, Y: -1.043110, Q -- W: 0.317378, X: -0.775730, Y: -0.249406, Z: -0.485088
[INFO] [1703833584.757284339] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833584.757814881] [move_group_interface]: Planning request accepted
[INFO] [1703833597.285134679] [move_group_interface]: Planning request aborted
[ERROR] [1703833597.285890470] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703833597.285918220] [RTXAgentClientNode]: move_arm  took 12528 ms
[ERROR] [1703833597.285941679] [RTXAgentClientNode]: Planning failed!
[INFO] [1703833597.285980470] [RTXAgentClientNode]: Action took 12862 ms
[INFO] [1703833597.786141096] [RTXAgentClientNode]: *** Iteration 8
[INFO] [1703833597.793386846] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833598.171956513] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833598.171994721] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833598.172008471] [RTXAgentClientNode]: Action server available after 385 ms
[ERROR] [1703833598.172042346] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833598.172048346] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833598.172055221] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833598.172060346] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833598.172065221] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833598.172070679] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703833598.172082346] [RTXAgentClientNode]: VLA error: 
[INFO] [1703833598.172090804] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996062, Target: 1.000000
[INFO] [1703833598.172549679] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036931, Target: 0.037000
[INFO] [1703833598.172629929] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833598.173072596] [move_group_interface]: Planning request accepted
[INFO] [1703833598.268165429] [move_group_interface]: Planning request complete!
[INFO] [1703833598.268998679] [move_group_interface]: time taken to generate plan: 0.0131187 seconds
[INFO] [1703833598.269021263] [RTXAgentClientNode]: move_hand 96 ms
[INFO] [1703833598.272903679] [move_group_interface]: Execute request accepted
[INFO] [1703833598.331297596] [move_group_interface]: Execute request success!
[INFO] [1703833598.331912763] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833598.332061804] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.074708, Y: -0.298708, Z: 0.076733, RPY --  R: -1.990765, P: 0.636373, Y: -1.043110, Q -- W: 0.578915, X: -0.606088, Y: 0.544626, Z: -0.029947
[INFO] [1703833598.332112763] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.074708, Y: -0.278708, Z: 0.056733, RPY --  R: -1.990765, P: 0.636373, Y: -1.043110, Q -- W: 0.317378, X: -0.775730, Y: -0.249406, Z: -0.485088
[INFO] [1703833598.332243096] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833598.335795721] [move_group_interface]: Planning request accepted
[INFO] [1703833610.944765963] [move_group_interface]: Planning request aborted
[ERROR] [1703833610.945645213] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703833610.945691004] [RTXAgentClientNode]: move_arm  took 12612 ms
[ERROR] [1703833610.945723754] [RTXAgentClientNode]: Planning failed!
[INFO] [1703833610.945809879] [RTXAgentClientNode]: Action took 13158 ms
[INFO] [1703833611.445908296] [RTXAgentClientNode]: *** Iteration 9
[INFO] [1703833611.449313880] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833611.865573630] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833611.865988505] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833611.866114296] [RTXAgentClientNode]: Action server available after 420 ms
[ERROR] [1703833611.866171838] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703833611.866345921] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833611.866388630] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833611.866424796] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833611.866461338] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833611.866499671] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833611.866539421] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833611.866590380] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996062, Target: 1.000000
[INFO] [1703833611.866879963] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036931, Target: 0.037000
[INFO] [1703833611.867261963] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833611.868762296] [move_group_interface]: Planning request accepted
[INFO] [1703833611.908939796] [move_group_interface]: Planning request complete!
[INFO] [1703833611.909916755] [move_group_interface]: time taken to generate plan: 0.0125907 seconds
[INFO] [1703833611.909977630] [RTXAgentClientNode]: move_hand 43 ms
[INFO] [1703833611.910518421] [move_group_interface]: Execute request accepted
[INFO] [1703833611.968877671] [move_group_interface]: Execute request success!
[INFO] [1703833611.969069546] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833611.969197880] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.074708, Y: -0.298708, Z: 0.076733, RPY --  R: -1.990765, P: 0.636373, Y: -1.043110, Q -- W: 0.578915, X: -0.606088, Y: 0.544626, Z: -0.029947
[INFO] [1703833611.969216588] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.074708, Y: -0.278708, Z: 0.056733, RPY --  R: -1.990765, P: 0.636373, Y: -1.043110, Q -- W: 0.317378, X: -0.775730, Y: -0.249406, Z: -0.485088
[INFO] [1703833611.969287338] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833611.969774880] [move_group_interface]: Planning request accepted
[INFO] [1703833624.579824719] [move_group_interface]: Planning request aborted
[ERROR] [1703833624.580871511] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703833624.580916344] [RTXAgentClientNode]: move_arm  took 12611 ms
[ERROR] [1703833624.580951636] [RTXAgentClientNode]: Planning failed!
[INFO] [1703833624.580964219] [RTXAgentClientNode]: Action took 13135 ms
[INFO] [1703833625.082687136] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[INFO] [1703833625.083357969] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[ros2run]: Segmentation fault
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=10
[INFO] [1703833769.618611383] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 1.58 seconds
[INFO] [1703833769.618703633] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703833769.863054925] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703833769.878588925] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703833769.885774800] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703833770.138948050] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.12 seconds
[INFO] [1703833770.138979550] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703833770.254804134] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703833770.388494300] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703833770.388992092] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703833770.389003800] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703833770.389198050] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703833770.389364300] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703833770.400720884] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703833770.400755467] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703833770.437823176] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703833770.437956259] [RTXAgentClientNode]: Waiting...
[INFO] [1703833780.101441930] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833780.453522347] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833780.453615805] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833780.453638597] [RTXAgentClientNode]: Action server available after 10015 ms
[ERROR] [1703833780.453668180] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833780.453673264] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833780.453681347] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833780.453688639] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833780.453693347] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833780.453704014] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703833780.453852764] [RTXAgentClientNode]: VLA error: 
[INFO] [1703833780.454344097] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703833780.456363722] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703833780.504744055] [RTXAgentClientNode]: HAND OPENNESS -- Current: -1.114286, Target: 1.000000
[INFO] [1703833780.504785097] [RTXAgentClientNode]: FINGER JOINT -- Current: -0.000000, Target: 0.019500
[INFO] [1703833780.504914430] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833780.505711805] [move_group_interface]: Planning request accepted
[INFO] [1703833780.549693097] [move_group_interface]: Planning request complete!
[INFO] [1703833780.550666472] [move_group_interface]: time taken to generate plan: 0.005917 seconds
[INFO] [1703833780.551458805] [RTXAgentClientNode]: move_hand 97 ms
[INFO] [1703833780.554644472] [move_group_interface]: Execute request accepted
[INFO] [1703833798.550606383] [move_group_interface]: Execute request success!
[INFO] [1703833798.551586466] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833798.551751841] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.308721, Y: 0.000327, Z: 0.247229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: -0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703833798.551775133] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.308721, Y: 0.020327, Z: 0.227229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: 0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703833798.551860050] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833798.555281925] [move_group_interface]: Planning request accepted
[INFO] [1703833798.671895758] [move_group_interface]: Planning request complete!
[INFO] [1703833798.671924925] [move_group_interface]: time taken to generate plan: 0.0198815 seconds
[INFO] [1703833798.672510883] [RTXAgentClientNode]: move_arm  took 120 ms
[INFO] [1703833798.673567300] [move_group_interface]: Execute request accepted
[INFO] [1703833802.347005635] [move_group_interface]: Execute request success!
[INFO] [1703833802.348038468] [RTXAgentClientNode]: Action took 31909 ms
[INFO] [1703833803.348270427] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703833803.350073969] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833803.720311260] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833803.720355969] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833803.720368344] [RTXAgentClientNode]: Action server available after 372 ms
[ERROR] [1703833803.720401635] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833803.720412885] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833803.720438927] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703833803.720467760] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833803.720526344] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833803.720576094] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833803.720622677] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833803.720670552] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.002281, Target: 1.000000
[INFO] [1703833803.720723760] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019460, Target: 0.037000
[INFO] [1703833803.720942344] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833803.721520927] [move_group_interface]: Planning request accepted
[INFO] [1703833803.753154344] [move_group_interface]: Planning request complete!
[INFO] [1703833803.753408010] [move_group_interface]: time taken to generate plan: 0.00679296 seconds
[INFO] [1703833803.753458844] [RTXAgentClientNode]: move_hand 32 ms
[INFO] [1703833803.754178969] [move_group_interface]: Execute request accepted
[INFO] [1703833820.739812213] [move_group_interface]: Execute request success!
[INFO] [1703833820.740509838] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833820.742563004] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291680, Y: 0.020265, Z: 0.248896, RPY --  R: 0.000937, P: 0.900872, Y: 0.002362, Q -- W: 0.900257, X: -0.000092, Y: 0.435358, Z: 0.000859
[INFO] [1703833820.742625004] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291680, Y: 0.040265, Z: 0.228896, RPY --  R: 0.000937, P: 0.900872, Y: 0.002362, Q -- W: 0.900256, X: 0.000936, Y: 0.435357, Z: 0.001267
[INFO] [1703833820.742905504] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833820.744685004] [move_group_interface]: Planning request accepted
[INFO] [1703833820.855966796] [move_group_interface]: Planning request complete!
[INFO] [1703833820.856359129] [move_group_interface]: time taken to generate plan: 0.0193393 seconds
[INFO] [1703833820.856393046] [RTXAgentClientNode]: move_arm  took 115 ms
[INFO] [1703833820.857349421] [move_group_interface]: Execute request accepted
[INFO] [1703833824.492257256] [move_group_interface]: Execute request success!
[INFO] [1703833824.493013839] [RTXAgentClientNode]: Action took 21145 ms
[INFO] [1703833825.501723381] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703833825.502215465] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833825.753540840] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833825.753577048] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833825.753595882] [RTXAgentClientNode]: Action server available after 251 ms
[ERROR] [1703833825.753617715] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833825.753627798] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833825.753635882] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833825.753647048] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833825.753651673] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833825.753663965] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703833825.753682423] [RTXAgentClientNode]: VLA error: 
[INFO] [1703833825.753726007] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.994526, Target: 1.000000
[INFO] [1703833825.753735257] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036904, Target: 0.037000
[INFO] [1703833825.753842465] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833825.755777173] [move_group_interface]: Planning request accepted
[INFO] [1703833825.813456840] [move_group_interface]: Planning request complete!
[INFO] [1703833825.814504007] [move_group_interface]: time taken to generate plan: 0.0142386 seconds
[INFO] [1703833825.814533590] [RTXAgentClientNode]: move_hand 60 ms
[INFO] [1703833825.814974590] [move_group_interface]: Execute request accepted
[INFO] [1703833825.879338715] [move_group_interface]: Execute request success!
[INFO] [1703833825.879931965] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833825.879983215] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.274559, Y: 0.040160, Z: 0.250416, RPY --  R: 0.005702, P: 0.901123, Y: 0.005691, Q -- W: 0.900199, X: 0.001327, Y: 0.435475, Z: 0.001320
[INFO] [1703833825.879999590] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.274559, Y: 0.060160, Z: 0.230416, RPY --  R: 0.005702, P: 0.901123, Y: 0.005691, Q -- W: 0.900192, X: 0.003806, Y: 0.435460, Z: 0.003803
[INFO] [1703833825.880071715] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833825.881778257] [move_group_interface]: Planning request accepted
[INFO] [1703833825.998073090] [move_group_interface]: Planning request complete!
[INFO] [1703833825.998829965] [move_group_interface]: time taken to generate plan: 0.0191508 seconds
[INFO] [1703833825.998861007] [RTXAgentClientNode]: move_arm  took 118 ms
[INFO] [1703833825.999301507] [move_group_interface]: Execute request accepted
[INFO] [1703833829.523023508] [move_group_interface]: Execute request success!
[INFO] [1703833829.523596300] [RTXAgentClientNode]: Action took 4021 ms
[INFO] [1703833830.523726926] [RTXAgentClientNode]: *** Iteration 3
[INFO] [1703833830.524093592] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833830.829080426] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833830.829109342] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833830.829121342] [RTXAgentClientNode]: Action server available after 305 ms
[ERROR] [1703833830.830355592] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833830.830388842] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833830.830396801] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833830.830405884] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833830.830410217] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833830.830418009] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833830.830437842] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.994526, Target: 1.000000
[INFO] [1703833830.830445592] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036904, Target: 0.037000
[INFO] [1703833830.830541301] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833830.831882301] [move_group_interface]: Planning request accepted
[ERROR] [1703833830.829389967] [RTXAgentClientNode]: VLA error: 
[INFO] [1703833830.873808301] [move_group_interface]: Planning request complete!
[INFO] [1703833830.874810884] [move_group_interface]: time taken to generate plan: 0.0123368 seconds
[INFO] [1703833830.874833467] [RTXAgentClientNode]: move_hand 44 ms
[INFO] [1703833830.875667176] [move_group_interface]: Execute request accepted
[INFO] [1703833830.977372217] [move_group_interface]: Execute request success!
[INFO] [1703833830.978055009] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833830.978245342] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.257451, Y: 0.059932, Z: 0.252031, RPY --  R: 0.015228, P: 0.901879, Y: 0.015822, Q -- W: 0.900010, X: 0.003405, Y: 0.435839, Z: 0.003802
[INFO] [1703833830.978270634] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.257451, Y: 0.079932, Z: 0.232031, RPY --  R: 0.015228, P: 0.901879, Y: 0.015822, Q -- W: 0.899958, X: 0.010300, Y: 0.435731, Z: 0.010438
[INFO] [1703833830.978359592] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833830.978968092] [move_group_interface]: Planning request accepted
[INFO] [1703833831.071860676] [move_group_interface]: Planning request complete!
[INFO] [1703833831.072251217] [move_group_interface]: time taken to generate plan: 0.0160669 seconds
[INFO] [1703833831.072269634] [RTXAgentClientNode]: move_arm  took 94 ms
[INFO] [1703833831.075021592] [move_group_interface]: Execute request accepted
[INFO] [1703833834.429626761] [move_group_interface]: Execute request success!
[INFO] [1703833834.430454094] [RTXAgentClientNode]: Action took 3906 ms
[INFO] [1703833835.430905178] [RTXAgentClientNode]: *** Iteration 4
[INFO] [1703833835.431488886] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833835.781828761] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833835.781862178] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833835.781879220] [RTXAgentClientNode]: Action server available after 350 ms
[ERROR] [1703833835.781918095] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833835.781926470] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833835.781952845] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833835.781960595] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833835.781968428] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833835.781975886] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833835.781999595] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.994526, Target: 1.000000
[INFO] [1703833835.782035636] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036904, Target: 0.037000
[INFO] [1703833835.782119345] [move_group_interface]: MoveGroup action client/server ready
[ERROR] [1703833835.782410428] [RTXAgentClientNode]: VLA error: 
[INFO] [1703833835.782819928] [move_group_interface]: Planning request accepted
[INFO] [1703833835.822223220] [move_group_interface]: Planning request complete!
[INFO] [1703833835.822790345] [move_group_interface]: time taken to generate plan: 0.012666 seconds
[INFO] [1703833835.822808970] [RTXAgentClientNode]: move_hand 40 ms
[INFO] [1703833835.823679970] [move_group_interface]: Execute request accepted
[INFO] [1703833835.930914845] [move_group_interface]: Execute request success!
[INFO] [1703833835.931863136] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833835.932078845] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.240341, Y: 0.079215, Z: 0.253683, RPY --  R: 0.044381, P: 0.900528, Y: 0.043469, Q -- W: 0.900108, X: 0.010517, Y: 0.435427, Z: 0.009907
[INFO] [1703833835.932123136] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.240341, Y: 0.099215, Z: 0.233683, RPY --  R: 0.044381, P: 0.900528, Y: 0.043469, Q -- W: 0.899688, X: 0.029428, Y: 0.434559, Z: 0.029216
[INFO] [1703833835.932270845] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833835.935669761] [move_group_interface]: Planning request accepted
[INFO] [1703833841.462814458] [move_group_interface]: Planning request complete!
[INFO] [1703833841.463365541] [move_group_interface]: time taken to generate plan: 5.03873 seconds
[INFO] [1703833841.463602916] [RTXAgentClientNode]: move_arm  took 5530 ms
[INFO] [1703833841.466307833] [move_group_interface]: Execute request accepted
[INFO] [1703833868.232702846] [move_group_interface]: Execute request success!
[INFO] [1703833868.233194596] [RTXAgentClientNode]: Action took 32801 ms
[INFO] [1703833869.233343930] [RTXAgentClientNode]: *** Iteration 5
[INFO] [1703833869.234293846] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833869.682138347] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833869.682660555] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833869.682746055] [RTXAgentClientNode]: Action server available after 449 ms
[ERROR] [1703833869.682910847] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703833869.682981513] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833869.683010263] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833869.683043388] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833869.683066930] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833869.683089222] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833869.683139638] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833869.731214263] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.994526, Target: 1.000000
[INFO] [1703833869.731478472] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036904, Target: 0.037000
[INFO] [1703833869.731770055] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833869.732542138] [move_group_interface]: Planning request accepted
[INFO] [1703833869.795619263] [move_group_interface]: Planning request complete!
[INFO] [1703833869.797781388] [move_group_interface]: time taken to generate plan: 0.0232069 seconds
[INFO] [1703833869.797839430] [RTXAgentClientNode]: move_hand 114 ms
[INFO] [1703833869.798865597] [move_group_interface]: Execute request accepted
[INFO] [1703833869.861349305] [move_group_interface]: Execute request success!
[INFO] [1703833869.861784013] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833869.861980388] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.148922, Y: -0.192515, Z: 0.142718, RPY --  R: -2.115517, P: 0.590582, Y: -0.467085, Q -- W: 0.515512, X: -0.777862, Y: 0.331866, Z: 0.137995
[INFO] [1703833869.862071013] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.148922, Y: -0.172515, Z: 0.122718, RPY --  R: -2.115517, P: 0.590582, Y: -0.467085, Q -- W: 0.398156, X: -0.843975, Y: -0.053942, Z: -0.355342
[INFO] [1703833869.862175597] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833869.864112222] [move_group_interface]: Planning request accepted
[INFO] [1703833882.561050380] [move_group_interface]: Planning request aborted
[ERROR] [1703833882.561455338] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703833882.561485088] [RTXAgentClientNode]: move_arm  took 12699 ms
[ERROR] [1703833882.561503172] [RTXAgentClientNode]: Planning failed!
[INFO] [1703833882.561514463] [RTXAgentClientNode]: Action took 13327 ms
[INFO] [1703833883.561606172] [RTXAgentClientNode]: *** Iteration 6
[INFO] [1703833883.562672756] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833883.873698297] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833883.873843214] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833883.873915547] [RTXAgentClientNode]: Action server available after 312 ms
[ERROR] [1703833883.874044756] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833883.874136756] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833883.874601881] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833883.874630214] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833883.874649839] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833883.874667756] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833883.874703631] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.994526, Target: 1.000000
[INFO] [1703833883.874721881] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036904, Target: 0.037000
[INFO] [1703833883.874994964] [move_group_interface]: MoveGroup action client/server ready
[ERROR] [1703833883.874067964] [RTXAgentClientNode]: VLA error: 
[INFO] [1703833883.876836047] [move_group_interface]: Planning request accepted
[INFO] [1703833883.933862964] [move_group_interface]: Planning request complete!
[INFO] [1703833883.934211006] [move_group_interface]: time taken to generate plan: 0.00393679 seconds
[INFO] [1703833883.934274631] [RTXAgentClientNode]: move_hand 59 ms
[INFO] [1703833883.935409172] [move_group_interface]: Execute request accepted
[INFO] [1703833883.986805922] [move_group_interface]: Execute request success!
[INFO] [1703833883.986928922] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833883.987474631] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.148920, Y: -0.192506, Z: 0.142723, RPY --  R: -2.115482, P: 0.590442, Y: -0.467047, Q -- W: 0.515520, X: -0.777884, Y: 0.331825, Z: 0.137940
[INFO] [1703833883.987509672] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.148920, Y: -0.172506, Z: 0.122723, RPY --  R: -2.115482, P: 0.590442, Y: -0.467047, Q -- W: 0.398201, X: -0.843979, Y: -0.053956, Z: -0.355281
[INFO] [1703833883.987699297] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833883.988146631] [move_group_interface]: Planning request accepted
^C[INFO] [1703833894.326689302] [rclcpp]: signal_handler(signum=2)
Killed
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=10
[INFO] [1703833937.167711753] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.06 seconds
[INFO] [1703833937.167774753] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703833937.269166753] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703833937.278166086] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703833937.281661378] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703833937.416086794] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0 seconds
[INFO] [1703833937.416114836] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703833937.492338169] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703833937.581707169] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703833937.582155419] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703833937.582166669] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703833937.582349794] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703833937.582515794] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703833937.600649628] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703833937.600690711] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703833937.634613628] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703833937.634726961] [RTXAgentClientNode]: Waiting...
[INFO] [1703833946.157912673] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833946.534326424] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833946.534360257] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833946.534370590] [RTXAgentClientNode]: Action server available after 8899 ms
[ERROR] [1703833946.534393715] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833946.534407299] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833946.534413132] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833946.534416924] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833946.534421257] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833946.534425757] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703833946.534455382] [RTXAgentClientNode]: VLA error: 
[INFO] [1703833946.537146757] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703833946.538878132] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703833946.549892757] [RTXAgentClientNode]: HAND OPENNESS -- Current: -1.114286, Target: 1.000000
[INFO] [1703833946.549993132] [RTXAgentClientNode]: FINGER JOINT -- Current: -0.000000, Target: 0.019500
[INFO] [1703833946.550874757] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833946.551342715] [move_group_interface]: Planning request accepted
[INFO] [1703833946.617892924] [move_group_interface]: Planning request complete!
[INFO] [1703833946.618406299] [move_group_interface]: time taken to generate plan: 0.0234517 seconds
[INFO] [1703833946.618439299] [RTXAgentClientNode]: move_hand 83 ms
[INFO] [1703833946.619239632] [move_group_interface]: Execute request accepted
[INFO] [1703833964.608817876] [move_group_interface]: Execute request success!
[INFO] [1703833964.609295793] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833964.609456960] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.308721, Y: 0.000327, Z: 0.247229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: -0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703833964.609481501] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.308721, Y: 0.020327, Z: 0.227229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: 0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703833964.609590918] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833964.610611210] [move_group_interface]: Planning request accepted
[INFO] [1703833964.741078460] [move_group_interface]: Planning request complete!
[INFO] [1703833964.742365793] [move_group_interface]: time taken to generate plan: 0.0101837 seconds
[INFO] [1703833964.742734126] [RTXAgentClientNode]: move_arm  took 133 ms
[INFO] [1703833964.743407210] [move_group_interface]: Execute request accepted
[INFO] [1703833968.686752337] [move_group_interface]: Execute request success!
[INFO] [1703833968.687070128] [RTXAgentClientNode]: Action took 31054 ms
[INFO] [1703833969.687214170] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703833969.688389129] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833970.151865087] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833970.152165837] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833970.152221004] [RTXAgentClientNode]: Action server available after 464 ms
[ERROR] [1703833970.152319879] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833970.152362921] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833970.152281587] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703833970.152432587] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833970.152463546] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833970.152493962] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833970.152516879] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833970.152566171] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.003065, Target: 1.000000
[INFO] [1703833970.152595337] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019446, Target: 0.037000
[INFO] [1703833970.152752462] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833970.541538879] [move_group_interface]: Planning request accepted
[INFO] [1703833970.610948212] [move_group_interface]: Planning request complete!
[INFO] [1703833970.611022671] [move_group_interface]: time taken to generate plan: 0.0259074 seconds
[INFO] [1703833970.611051504] [RTXAgentClientNode]: move_hand 458 ms
[INFO] [1703833970.614568754] [move_group_interface]: Execute request accepted
[INFO] [1703833987.226889720] [move_group_interface]: Execute request success!
[INFO] [1703833987.228699595] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833987.228773804] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291674, Y: 0.020337, Z: 0.248870, RPY --  R: 0.001381, P: 0.900479, Y: 0.000990, Q -- W: 0.900343, X: 0.000406, Y: 0.435181, Z: 0.000145
[INFO] [1703833987.228837637] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291674, Y: 0.040337, Z: 0.228870, RPY --  R: 0.001381, P: 0.900479, Y: 0.000990, Q -- W: 0.900342, X: 0.000837, Y: 0.435181, Z: 0.000746
[INFO] [1703833987.228948804] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833987.230832095] [move_group_interface]: Planning request accepted
[INFO] [1703833987.351541262] [move_group_interface]: Planning request complete!
[INFO] [1703833987.351554887] [move_group_interface]: time taken to generate plan: 0.017326 seconds
[INFO] [1703833987.351591845] [RTXAgentClientNode]: move_arm  took 122 ms
[INFO] [1703833987.352050095] [move_group_interface]: Execute request accepted
[INFO] [1703833990.976101625] [move_group_interface]: Execute request success!
[INFO] [1703833990.976839458] [RTXAgentClientNode]: Action took 21289 ms
[INFO] [1703833991.976956292] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703833991.977815083] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833992.330125250] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833992.330177625] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703833992.330197542] [RTXAgentClientNode]: Action server available after 353 ms
[ERROR] [1703833992.330239667] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833992.330270042] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833992.330274792] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703833992.330387292] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833992.330400000] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833992.330422167] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833992.330438584] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833992.330464542] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996389, Target: 1.000000
[INFO] [1703833992.330481625] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036937, Target: 0.037000
[INFO] [1703833992.330605209] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833992.332374792] [move_group_interface]: Planning request accepted
[INFO] [1703833992.381519292] [move_group_interface]: Planning request complete!
[INFO] [1703833992.382545375] [move_group_interface]: time taken to generate plan: 0.0143596 seconds
[INFO] [1703833992.382599417] [RTXAgentClientNode]: move_hand 52 ms
[INFO] [1703833992.383708209] [move_group_interface]: Execute request accepted
[INFO] [1703833992.439395292] [move_group_interface]: Execute request success!
[INFO] [1703833992.440270834] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833992.440331834] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.274600, Y: 0.040297, Z: 0.250523, RPY --  R: 0.001995, P: 0.899908, Y: 0.002067, Q -- W: 0.900467, X: 0.000448, Y: 0.434925, Z: 0.000497
[INFO] [1703833992.440348000] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.274600, Y: 0.060297, Z: 0.230523, RPY --  R: 0.001995, P: 0.899908, Y: 0.002067, Q -- W: 0.900466, X: 0.001348, Y: 0.434923, Z: 0.001364
[INFO] [1703833992.440452959] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833992.440837834] [move_group_interface]: Planning request accepted
[INFO] [1703833992.556952834] [move_group_interface]: Planning request complete!
[INFO] [1703833992.557243250] [move_group_interface]: time taken to generate plan: 0.0184034 seconds
[INFO] [1703833992.557374834] [RTXAgentClientNode]: move_arm  took 117 ms
[INFO] [1703833992.557882334] [move_group_interface]: Execute request accepted
[INFO] [1703833996.130416252] [move_group_interface]: Execute request success!
[INFO] [1703833996.131171502] [RTXAgentClientNode]: Action took 4154 ms
[INFO] [1703833997.131343377] [RTXAgentClientNode]: *** Iteration 3
[INFO] [1703833997.138033544] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703833997.450905003] [RTXAgentClientNode]: Feedback callback
[INFO] [1703833997.450976003] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[ERROR] [1703833997.451146711] [RTXAgentClientNode]: VLA error: 
[INFO] [1703833997.451195919] [RTXAgentClientNode]: Action server available after 319 ms
[ERROR] [1703833997.451203461] [RTXAgentClientNode]: move_hand began
[ERROR] [1703833997.451208003] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703833997.451213086] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703833997.451217419] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703833997.451221961] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703833997.451226503] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703833997.451241086] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996389, Target: 1.000000
[INFO] [1703833997.451273669] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036937, Target: 0.037000
[INFO] [1703833997.451357628] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833997.452769711] [move_group_interface]: Planning request accepted
[INFO] [1703833997.492700753] [move_group_interface]: Planning request complete!
[INFO] [1703833997.492759878] [move_group_interface]: time taken to generate plan: 0.0143825 seconds
[INFO] [1703833997.492779044] [RTXAgentClientNode]: move_hand 41 ms
[INFO] [1703833997.495965461] [move_group_interface]: Execute request accepted
[INFO] [1703833997.598929378] [move_group_interface]: Execute request success!
[INFO] [1703833997.599900503] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703833997.599964794] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.257575, Y: 0.060164, Z: 0.252155, RPY --  R: 0.006736, P: 0.900714, Y: 0.007242, Q -- W: 0.900286, X: 0.001456, Y: 0.435293, Z: 0.001794
[INFO] [1703833997.599979503] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.257575, Y: 0.080164, Z: 0.232155, RPY --  R: 0.006736, P: 0.900714, Y: 0.007242, Q -- W: 0.900275, X: 0.004608, Y: 0.435271, Z: 0.004726
[INFO] [1703833997.600122669] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703833997.601035878] [move_group_interface]: Planning request accepted
[INFO] [1703833997.711286794] [move_group_interface]: Planning request complete!
[INFO] [1703833997.713246836] [move_group_interface]: time taken to generate plan: 0.0218799 seconds
[INFO] [1703833997.713281919] [RTXAgentClientNode]: move_arm  took 113 ms
[INFO] [1703833997.714191086] [move_group_interface]: Execute request accepted
[INFO] [1703834000.955985254] [move_group_interface]: Execute request success!
[INFO] [1703834000.956735838] [RTXAgentClientNode]: Action took 3825 ms
[INFO] [1703834001.956832630] [RTXAgentClientNode]: *** Iteration 4
[INFO] [1703834001.959361005] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834002.290007297] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834002.290105463] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834002.290167338] [RTXAgentClientNode]: Action server available after 333 ms
[ERROR] [1703834002.290245255] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703834002.290502880] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834002.290725547] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834002.290761672] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834002.290778755] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834002.290801713] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834002.290843880] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834002.290896463] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996389, Target: 1.000000
[INFO] [1703834002.290944172] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036937, Target: 0.037000
[INFO] [1703834002.291247838] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834002.292653005] [move_group_interface]: Planning request accepted
[INFO] [1703834002.332691922] [move_group_interface]: Planning request complete!
[INFO] [1703834002.333092130] [move_group_interface]: time taken to generate plan: 0.012243 seconds
[INFO] [1703834002.333115547] [RTXAgentClientNode]: move_hand 42 ms
[INFO] [1703834002.333529255] [move_group_interface]: Execute request accepted
[INFO] [1703834002.386856088] [move_group_interface]: Execute request success!
[INFO] [1703834002.387789922] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834002.388039880] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.240547, Y: 0.079787, Z: 0.253727, RPY --  R: 0.021638, P: 0.900762, Y: 0.021481, Q -- W: 0.900227, X: 0.005064, Y: 0.435363, Z: 0.004960
[INFO] [1703834002.388193297] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.240547, Y: 0.099787, Z: 0.233727, RPY --  R: 0.021638, P: 0.900762, Y: 0.021481, Q -- W: 0.900126, X: 0.014414, Y: 0.435154, Z: 0.014378
[INFO] [1703834002.388610505] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834002.389367588] [move_group_interface]: Planning request accepted
[INFO] [1703834002.499851213] [move_group_interface]: Planning request complete!
[INFO] [1703834002.500170797] [move_group_interface]: time taken to generate plan: 0.0247776 seconds
[INFO] [1703834002.500294505] [RTXAgentClientNode]: move_arm  took 112 ms
[INFO] [1703834002.500778963] [move_group_interface]: Execute request accepted
[INFO] [1703834005.612534798] [move_group_interface]: Execute request success!
[INFO] [1703834005.613424840] [RTXAgentClientNode]: Action took 3656 ms
[INFO] [1703834006.613602840] [RTXAgentClientNode]: *** Iteration 5
[INFO] [1703834006.614947174] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834006.921156965] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834006.921237174] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834006.921272465] [RTXAgentClientNode]: Action server available after 307 ms
[ERROR] [1703834006.921303507] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834006.921314132] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834006.921342840] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703834006.921676132] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834006.921840965] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834006.921940257] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834006.922056549] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834006.922193924] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996389, Target: 1.000000
[INFO] [1703834006.922305757] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036937, Target: 0.037000
[INFO] [1703834006.922636882] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834006.923762965] [move_group_interface]: Planning request accepted
[INFO] [1703834006.983879007] [move_group_interface]: Planning request complete!
[INFO] [1703834006.984597757] [move_group_interface]: time taken to generate plan: 0.0135157 seconds
[INFO] [1703834006.984635132] [RTXAgentClientNode]: move_hand 62 ms
[INFO] [1703834006.985035591] [move_group_interface]: Execute request accepted
[INFO] [1703834007.037778049] [move_group_interface]: Execute request success!
[INFO] [1703834007.038779132] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834007.038841757] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.223421, Y: 0.098712, Z: 0.255395, RPY --  R: 0.061574, P: 0.899949, Y: 0.061855, Q -- W: 0.900015, X: 0.014262, Y: 0.435386, Z: 0.014449
[INFO] [1703834007.038859091] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.223421, Y: 0.118712, Z: 0.235395, RPY --  R: 0.061574, P: 0.899949, Y: 0.061855, Q -- W: 0.899187, X: 0.041148, Y: 0.433671, Z: 0.041213
[INFO] [1703834007.038940674] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834007.039198174] [move_group_interface]: Planning request accepted
[INFO] [1703834012.588201260] [move_group_interface]: Planning request complete!
[INFO] [1703834012.597039010] [move_group_interface]: time taken to generate plan: 5.02199 seconds
[INFO] [1703834012.597110010] [RTXAgentClientNode]: move_arm  took 5558 ms
[INFO] [1703834012.598356343] [move_group_interface]: Execute request accepted
^C[INFO] [1703834035.790377382] [rclcpp]: signal_handler(signum=2)
^C[INFO] [1703834036.632744007] [rclcpp]: signal_handler(signum=2)
^[[AKilled
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=10
[INFO] [1703834104.659279969] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 3.74 seconds
[INFO] [1703834104.659410636] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703834104.810163178] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703834104.821105469] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703834104.829739636] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703834105.182578011] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.2 seconds
[INFO] [1703834105.182621844] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703834105.350695428] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703834105.518458178] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703834105.518890303] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703834105.518901845] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703834105.519585053] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703834105.520203595] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703834105.528007803] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703834105.528062553] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703834105.569929220] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703834105.570117636] [RTXAgentClientNode]: Waiting...
[INFO] [1703834107.260927762] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834107.553813137] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834107.553976262] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834107.554004179] [RTXAgentClientNode]: Action server available after 1983 ms
[ERROR] [1703834107.554140179] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703834107.554237929] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834107.554300637] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834107.554345596] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834107.554394012] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834107.554432262] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834107.554490137] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834107.555618637] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703834107.557077012] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703834107.604205929] [RTXAgentClientNode]: HAND OPENNESS -- Current: -1.114286, Target: 1.000000
[INFO] [1703834107.604304054] [RTXAgentClientNode]: FINGER JOINT -- Current: -0.000000, Target: 0.019500
[INFO] [1703834107.604409971] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834107.605288262] [move_group_interface]: Planning request accepted
[INFO] [1703834107.677539596] [move_group_interface]: Planning request complete!
[INFO] [1703834107.678166387] [move_group_interface]: time taken to generate plan: 0.0190685 seconds
[INFO] [1703834107.678209262] [RTXAgentClientNode]: move_hand 123 ms
[INFO] [1703834107.679075387] [move_group_interface]: Execute request accepted
[INFO] [1703834125.690723632] [move_group_interface]: Execute request success!
[INFO] [1703834125.691336007] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834125.691462382] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.308721, Y: 0.000327, Z: 0.247229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: -0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703834125.691485215] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.308721, Y: 0.020327, Z: 0.227229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: 0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703834125.691555340] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834125.697531048] [move_group_interface]: Planning request accepted
[INFO] [1703834125.811098798] [move_group_interface]: Planning request complete!
[INFO] [1703834125.811806840] [move_group_interface]: time taken to generate plan: 0.0191317 seconds
[INFO] [1703834125.811848173] [RTXAgentClientNode]: move_arm  took 120 ms
[INFO] [1703834125.812694632] [move_group_interface]: Execute request accepted
[INFO] [1703834129.494602842] [move_group_interface]: Execute request success!
[INFO] [1703834129.498048758] [RTXAgentClientNode]: Action took 23927 ms
[INFO] [1703834131.498159343] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703834131.498609551] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834131.918160801] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834131.918303301] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[ERROR] [1703834131.918407176] [RTXAgentClientNode]: VLA error: 
[INFO] [1703834131.918770885] [RTXAgentClientNode]: Action server available after 420 ms
[ERROR] [1703834131.919071510] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834131.919143093] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834131.919193926] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834131.919256010] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834131.919329385] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834131.919379426] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834131.919458926] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.002872, Target: 1.000000
[INFO] [1703834131.919536926] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019450, Target: 0.037000
[INFO] [1703834131.919696135] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834131.920972468] [move_group_interface]: Planning request accepted
[INFO] [1703834131.977267551] [move_group_interface]: Planning request complete!
[INFO] [1703834131.979232385] [move_group_interface]: time taken to generate plan: 0.0256781 seconds
[INFO] [1703834131.979314385] [RTXAgentClientNode]: move_hand 59 ms
[INFO] [1703834131.980236968] [move_group_interface]: Execute request accepted
[INFO] [1703834148.998150378] [move_group_interface]: Execute request success!
[INFO] [1703834148.998375837] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834148.998710087] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291627, Y: 0.020246, Z: 0.248826, RPY --  R: 0.001248, P: 0.900204, Y: 0.002857, Q -- W: 0.900402, X: -0.000059, Y: 0.435058, Z: 0.001015
[INFO] [1703834148.998920837] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291627, Y: 0.040246, Z: 0.228826, RPY --  R: 0.001248, P: 0.900204, Y: 0.002857, Q -- W: 0.900401, X: 0.001183, Y: 0.435056, Z: 0.001557
[INFO] [1703834148.999289337] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834149.001672670] [move_group_interface]: Planning request accepted
[INFO] [1703834149.117616003] [move_group_interface]: Planning request complete!
[INFO] [1703834149.124837378] [move_group_interface]: time taken to generate plan: 0.0160289 seconds
[INFO] [1703834149.124865337] [RTXAgentClientNode]: move_arm  took 126 ms
[INFO] [1703834149.125276920] [move_group_interface]: Execute request accepted
[INFO] [1703834152.916121422] [move_group_interface]: Execute request success!
[INFO] [1703834152.917035964] [RTXAgentClientNode]: Action took 21418 ms
[INFO] [1703834154.917549465] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703834154.930459048] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834155.301454798] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834155.301606673] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[ERROR] [1703834155.301731715] [RTXAgentClientNode]: VLA error: 
[INFO] [1703834155.303379590] [RTXAgentClientNode]: Action server available after 385 ms
[ERROR] [1703834155.303905631] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834155.304077631] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834155.304351798] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834155.304378131] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834155.304387340] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834155.304411298] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834155.304465340] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.994676, Target: 1.000000
[INFO] [1703834155.304584840] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036907, Target: 0.037000
[INFO] [1703834155.304979048] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834155.306331923] [move_group_interface]: Planning request accepted
[INFO] [1703834155.377036548] [move_group_interface]: Planning request complete!
[INFO] [1703834155.377253423] [move_group_interface]: time taken to generate plan: 0.0139546 seconds
[INFO] [1703834155.377320965] [RTXAgentClientNode]: move_hand 72 ms
[INFO] [1703834155.377902881] [move_group_interface]: Execute request accepted
[INFO] [1703834155.430748423] [move_group_interface]: Execute request success!
[INFO] [1703834155.431264673] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834155.431459840] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.274439, Y: 0.040203, Z: 0.250371, RPY --  R: 0.005313, P: 0.899750, Y: 0.004804, Q -- W: 0.900499, X: 0.001347, Y: 0.434856, Z: 0.001008
[INFO] [1703834155.431514215] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.274439, Y: 0.060203, Z: 0.230371, RPY --  R: 0.005313, P: 0.899750, Y: 0.004804, Q -- W: 0.900493, X: 0.003436, Y: 0.434844, Z: 0.003318
[INFO] [1703834155.431632756] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834155.432646881] [move_group_interface]: Planning request accepted
[INFO] [1703834155.549521882] [move_group_interface]: Planning request complete!
[INFO] [1703834155.549539507] [move_group_interface]: time taken to generate plan: 0.0185828 seconds
[INFO] [1703834155.549620548] [RTXAgentClientNode]: move_arm  took 118 ms
[INFO] [1703834155.550071007] [move_group_interface]: Execute request accepted
[INFO] [1703834158.961794258] [move_group_interface]: Execute request success!
[INFO] [1703834158.962241050] [RTXAgentClientNode]: Action took 4044 ms
[INFO] [1703834160.962387467] [RTXAgentClientNode]: *** Iteration 3
[INFO] [1703834160.962796426] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834161.317529009] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834161.317618301] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[ERROR] [1703834161.317666384] [RTXAgentClientNode]: VLA error: 
[INFO] [1703834161.318196259] [RTXAgentClientNode]: Action server available after 355 ms
[ERROR] [1703834161.318247259] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834161.318274551] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834161.318280926] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834161.318296218] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834161.318306343] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834161.318310968] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834161.318332134] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.994676, Target: 1.000000
[INFO] [1703834161.318370009] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036907, Target: 0.037000
[INFO] [1703834161.318495884] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834161.319238509] [move_group_interface]: Planning request accepted
[INFO] [1703834161.364739509] [move_group_interface]: Planning request complete!
[INFO] [1703834161.365426884] [move_group_interface]: time taken to generate plan: 0.0132715 seconds
[INFO] [1703834161.365511718] [RTXAgentClientNode]: move_hand 47 ms
[INFO] [1703834161.370284843] [move_group_interface]: Execute request accepted
[INFO] [1703834161.424115843] [move_group_interface]: Execute request success!
[INFO] [1703834161.425192134] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834161.425331009] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.257311, Y: 0.059880, Z: 0.252045, RPY --  R: 0.015172, P: 0.899846, Y: 0.015812, Q -- W: 0.900453, X: 0.003392, Y: 0.434924, Z: 0.003820
[INFO] [1703834161.425368509] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.257311, Y: 0.079880, Z: 0.232045, RPY --  R: 0.015172, P: 0.899846, Y: 0.015812, Q -- W: 0.900400, X: 0.010269, Y: 0.434816, Z: 0.010418
[INFO] [1703834161.425767218] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834161.426517468] [move_group_interface]: Planning request accepted
[INFO] [1703834161.538677426] [move_group_interface]: Planning request complete!
[INFO] [1703834161.539645301] [move_group_interface]: time taken to generate plan: 0.0186955 seconds
[INFO] [1703834161.539694676] [RTXAgentClientNode]: move_arm  took 114 ms
[INFO] [1703834161.540149301] [move_group_interface]: Execute request accepted
[INFO] [1703834165.015192594] [move_group_interface]: Execute request success!
[INFO] [1703834165.015742886] [RTXAgentClientNode]: Action took 4053 ms
[INFO] [1703834167.015859720] [RTXAgentClientNode]: *** Iteration 4
[INFO] [1703834167.020742679] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834167.443436137] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834167.443808262] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834167.443883971] [RTXAgentClientNode]: Action server available after 427 ms
[ERROR] [1703834167.444443179] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834167.444463346] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834167.444481679] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834167.444505887] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834167.444031762] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703834167.444827346] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834167.444855929] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834167.444882512] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.994676, Target: 1.000000
[INFO] [1703834167.444892846] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036907, Target: 0.037000
[INFO] [1703834167.445010221] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834167.445944846] [move_group_interface]: Planning request accepted
[INFO] [1703834167.536313929] [move_group_interface]: Planning request complete!
[INFO] [1703834167.536911096] [move_group_interface]: time taken to generate plan: 0.0124456 seconds
[INFO] [1703834167.537057304] [RTXAgentClientNode]: move_hand 92 ms
[INFO] [1703834167.537688137] [move_group_interface]: Execute request accepted
[INFO] [1703834167.590248721] [move_group_interface]: Execute request success!
[INFO] [1703834167.590793846] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834167.590893429] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.240172, Y: 0.079160, Z: 0.253683, RPY --  R: 0.044007, P: 0.900103, Y: 0.043284, Q -- W: 0.900203, X: 0.010395, Y: 0.435234, Z: 0.009912
[INFO] [1703834167.590911096] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.240172, Y: 0.099160, Z: 0.233683, RPY --  R: 0.044007, P: 0.900103, Y: 0.043284, Q -- W: 0.899789, X: 0.029218, Y: 0.434376, Z: 0.029050
[INFO] [1703834167.591019887] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834167.591471221] [move_group_interface]: Planning request accepted
[INFO] [1703834173.072871292] [move_group_interface]: Planning request complete!
[INFO] [1703834173.073185376] [move_group_interface]: time taken to generate plan: 5.01806 seconds
[INFO] [1703834173.073240459] [RTXAgentClientNode]: move_arm  took 5483 ms
[INFO] [1703834173.076993542] [move_group_interface]: Execute request accepted
[INFO] [1703834194.202169386] [move_group_interface]: Execute request success!
[INFO] [1703834194.202988177] [RTXAgentClientNode]: Action took 27188 ms
[INFO] [1703834196.203253095] [RTXAgentClientNode]: *** Iteration 5
[INFO] [1703834196.222238220] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834196.543484845] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834196.543671512] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834196.543691595] [RTXAgentClientNode]: Action server available after 340 ms
[ERROR] [1703834196.543716595] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834196.543725178] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834196.543728887] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703834196.543759970] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834196.543768303] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834196.543775220] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834196.543791637] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834196.543822887] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.994676, Target: 1.000000
[INFO] [1703834196.543840053] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036907, Target: 0.037000
[INFO] [1703834196.543913637] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834196.545275512] [move_group_interface]: Planning request accepted
[INFO] [1703834196.581186845] [move_group_interface]: Planning request complete!
[INFO] [1703834196.581440803] [move_group_interface]: time taken to generate plan: 0.0118035 seconds
[INFO] [1703834196.581463220] [RTXAgentClientNode]: move_hand 37 ms
[INFO] [1703834196.581654345] [move_group_interface]: Execute request accepted
[INFO] [1703834196.634357220] [move_group_interface]: Execute request success!
[INFO] [1703834196.634804095] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834196.634963428] [RTXAgentClientNode]: Current (world frame) XYZ -- X: -0.138667, Y: -0.372537, Z: 0.211940, RPY --  R: -1.275504, P: 0.043743, Y: -1.823415, Q -- W: 0.502198, X: -0.350637, Y: 0.481334, Z: -0.627031
[INFO] [1703834196.635047345] [RTXAgentClientNode]: Target (world frame) XYZ -- X: -0.138667, Y: -0.352537, Z: 0.191940, RPY --  R: -1.275504, P: 0.043743, Y: -1.823415, Q -- W: 0.481610, X: -0.378419, Y: -0.459814, Z: -0.642979
[INFO] [1703834196.635170637] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834196.636740512] [move_group_interface]: Planning request accepted
[INFO] [1703834196.773682887] [move_group_interface]: Planning request complete!
[INFO] [1703834196.774224554] [move_group_interface]: time taken to generate plan: 0.0421619 seconds
[INFO] [1703834196.774530929] [RTXAgentClientNode]: move_arm  took 139 ms
[INFO] [1703834196.776123554] [move_group_interface]: Execute request accepted
^C[INFO] [1703834211.013527629] [rclcpp]: signal_handler(signum=2)
Killed
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=10 -p world_frame:=locobot_base_footprint
[INFO] [1703834277.277746049] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 7 seconds
[INFO] [1703834277.277794299] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703834277.415892841] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703834277.426296924] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703834277.435976882] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703834277.679669174] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.12 seconds
[INFO] [1703834277.679754716] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703834277.809920633] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703834277.940404174] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703834277.941069258] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703834277.941103174] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703834277.941499008] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703834277.942084674] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703834277.960241674] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703834277.960284633] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703834277.996047924] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703834277.996368299] [RTXAgentClientNode]: Waiting...
[INFO] [1703834277.996750258] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834278.340863008] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834278.340960299] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834278.341020049] [RTXAgentClientNode]: Action server available after 344 ms
[ERROR] [1703834278.341065758] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834278.341073633] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834278.341079966] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834278.341084216] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834278.341088716] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834278.341093466] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703834278.341296341] [RTXAgentClientNode]: VLA error: 
[INFO] [1703834278.341980716] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703834278.344269758] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703834278.354406425] [RTXAgentClientNode]: HAND OPENNESS -- Current: -1.114286, Target: 1.000000
[INFO] [1703834278.354439633] [RTXAgentClientNode]: FINGER JOINT -- Current: -0.000000, Target: 0.019500
[INFO] [1703834278.354529133] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834278.355205550] [move_group_interface]: Planning request accepted
[INFO] [1703834278.412919633] [move_group_interface]: Planning request complete!
[INFO] [1703834278.413351383] [move_group_interface]: time taken to generate plan: 0.0129047 seconds
[INFO] [1703834278.413528841] [RTXAgentClientNode]: move_hand 72 ms
[INFO] [1703834278.419351216] [move_group_interface]: Execute request accepted
[INFO] [1703834296.676826377] [move_group_interface]: Execute request success!
[INFO] [1703834296.677212086] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834296.677359294] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.308721, Y: 0.000327, Z: 0.247229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: -0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703834296.677381294] [RTXAgentClientNode]: Target (locobot_base_footprint frame) XYZ -- X: 0.308721, Y: 0.020327, Z: 0.227229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: 0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703834296.677480336] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834296.681951627] [move_group_interface]: Planning request accepted
[INFO] [1703834296.801061794] [move_group_interface]: Planning request complete!
[INFO] [1703834296.801416294] [move_group_interface]: time taken to generate plan: 0.0196165 seconds
[INFO] [1703834296.801476669] [RTXAgentClientNode]: move_arm  took 124 ms
[INFO] [1703834296.805636002] [move_group_interface]: Execute request accepted
[INFO] [1703834300.439274337] [move_group_interface]: Execute request success!
[INFO] [1703834300.439862671] [RTXAgentClientNode]: Action took 22443 ms
[INFO] [1703834302.439972463] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703834302.440655713] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834302.831924464] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834302.831963464] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834302.831981255] [RTXAgentClientNode]: Action server available after 391 ms
[ERROR] [1703834302.832028714] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834302.832035339] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834302.832166547] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834302.832101464] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703834302.832267047] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834302.832280505] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834302.832288422] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834302.832308089] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.001413, Target: 1.000000
[INFO] [1703834302.832324589] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019475, Target: 0.037000
[INFO] [1703834302.832420047] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834302.834194630] [move_group_interface]: Planning request accepted
[INFO] [1703834302.916057964] [move_group_interface]: Planning request complete!
[INFO] [1703834302.916408755] [move_group_interface]: time taken to generate plan: 0.0428496 seconds
[INFO] [1703834302.916570380] [RTXAgentClientNode]: move_hand 84 ms
[INFO] [1703834302.917889755] [move_group_interface]: Execute request accepted
[INFO] [1703834319.442141388] [move_group_interface]: Execute request success!
[INFO] [1703834319.442403471] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834319.443057138] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291435, Y: 0.020654, Z: 0.248785, RPY --  R: 0.000482, P: 0.899775, Y: 0.002518, Q -- W: 0.900496, X: -0.000331, Y: 0.434864, Z: 0.001029
[INFO] [1703834319.443283221] [RTXAgentClientNode]: Target (locobot_base_footprint frame) XYZ -- X: 0.291435, Y: 0.040654, Z: 0.228785, RPY --  R: 0.000482, P: 0.899775, Y: 0.002518, Q -- W: 0.900495, X: 0.000764, Y: 0.434863, Z: 0.001238
[INFO] [1703834319.443776305] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834319.449619471] [move_group_interface]: Planning request accepted
[INFO] [1703834319.580480388] [move_group_interface]: Planning request complete!
[INFO] [1703834319.581659972] [move_group_interface]: time taken to generate plan: 0.0257565 seconds
[INFO] [1703834319.581723097] [RTXAgentClientNode]: move_arm  took 139 ms
[INFO] [1703834319.582585513] [move_group_interface]: Execute request accepted
[INFO] [1703834323.228765626] [move_group_interface]: Execute request success!
[INFO] [1703834323.229627792] [RTXAgentClientNode]: Action took 20788 ms
[INFO] [1703834325.229732293] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703834325.243202627] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834325.707734543] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834325.707846002] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[ERROR] [1703834325.708001960] [RTXAgentClientNode]: VLA error: 
[INFO] [1703834325.708596460] [RTXAgentClientNode]: Action server available after 478 ms
[ERROR] [1703834325.709048418] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834325.709193085] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834325.709280043] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834325.709332252] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834325.709383293] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834325.709432835] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834325.709501043] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996252, Target: 1.000000
[INFO] [1703834325.709551210] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036934, Target: 0.037000
[INFO] [1703834325.709700127] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834325.712841293] [move_group_interface]: Planning request accepted
[INFO] [1703834325.783540835] [move_group_interface]: Planning request complete!
[INFO] [1703834325.783871544] [move_group_interface]: time taken to generate plan: 0.0128831 seconds
[INFO] [1703834325.783898835] [RTXAgentClientNode]: move_hand 74 ms
[INFO] [1703834325.784759377] [move_group_interface]: Execute request accepted
[INFO] [1703834325.836462919] [move_group_interface]: Execute request success!
[INFO] [1703834325.837343377] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834325.837406044] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.274083, Y: 0.040834, Z: 0.250309, RPY --  R: 0.004594, P: 0.900660, Y: 0.006696, Q -- W: 0.900300, X: 0.000611, Y: 0.435266, Z: 0.002014
[INFO] [1703834325.837425002] [RTXAgentClientNode]: Target (locobot_base_footprint frame) XYZ -- X: 0.274083, Y: 0.060834, Z: 0.230309, RPY --  R: 0.004594, P: 0.900660, Y: 0.006696, Q -- W: 0.900293, X: 0.003525, Y: 0.435252, Z: 0.004014
[INFO] [1703834325.837500627] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834325.838063960] [move_group_interface]: Planning request accepted
[INFO] [1703834325.957810460] [move_group_interface]: Planning request complete!
[INFO] [1703834325.958553044] [move_group_interface]: time taken to generate plan: 0.0376619 seconds
[INFO] [1703834325.959139294] [RTXAgentClientNode]: move_arm  took 121 ms
[INFO] [1703834325.960369669] [move_group_interface]: Execute request accepted
[INFO] [1703834329.597719545] [move_group_interface]: Execute request success!
[INFO] [1703834329.598610295] [RTXAgentClientNode]: Action took 4368 ms
[INFO] [1703834331.598774338] [RTXAgentClientNode]: *** Iteration 3
[INFO] [1703834331.600229338] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834331.883906005] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834331.883996505] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834331.884058463] [RTXAgentClientNode]: Action server available after 285 ms
[ERROR] [1703834331.884115088] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834331.884179755] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834331.884200546] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834331.884233421] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834331.884322546] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834331.884267505] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703834331.884458005] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834331.884511671] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996252, Target: 1.000000
[INFO] [1703834331.885744880] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036934, Target: 0.037000
[INFO] [1703834331.885931338] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834331.886662588] [move_group_interface]: Planning request accepted
[INFO] [1703834331.943226213] [move_group_interface]: Planning request complete!
[INFO] [1703834331.944140713] [move_group_interface]: time taken to generate plan: 0.0123883 seconds
[INFO] [1703834331.944172421] [RTXAgentClientNode]: move_hand 59 ms
[INFO] [1703834331.944524546] [move_group_interface]: Execute request accepted
[INFO] [1703834331.996676088] [move_group_interface]: Execute request success!
[INFO] [1703834331.997204796] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834331.997334463] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.256816, Y: 0.060813, Z: 0.251987, RPY --  R: 0.016776, P: 0.901091, Y: 0.018211, Q -- W: 0.900174, X: 0.003586, Y: 0.435492, Z: 0.004544
[INFO] [1703834331.997367796] [RTXAgentClientNode]: Target (locobot_base_footprint frame) XYZ -- X: 0.256816, Y: 0.080813, Z: 0.231987, RPY --  R: 0.016776, P: 0.901091, Y: 0.018211, Q -- W: 0.900107, X: 0.011515, Y: 0.435355, Z: 0.011849
[INFO] [1703834331.997606755] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834331.998110838] [move_group_interface]: Planning request accepted
[INFO] [1703834332.112814963] [move_group_interface]: Planning request complete!
[INFO] [1703834332.113552838] [move_group_interface]: time taken to generate plan: 0.0186002 seconds
[INFO] [1703834332.113658922] [RTXAgentClientNode]: move_arm  took 116 ms
[INFO] [1703834332.114263047] [move_group_interface]: Execute request accepted
[INFO] [1703834335.533600340] [move_group_interface]: Execute request success!
[INFO] [1703834335.534665465] [RTXAgentClientNode]: Action took 3935 ms
[INFO] [1703834337.534799299] [RTXAgentClientNode]: *** Iteration 4
[INFO] [1703834337.535397799] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834337.883093216] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834337.883132008] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834337.883152508] [RTXAgentClientNode]: Action server available after 348 ms
[ERROR] [1703834337.883191216] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834337.883202424] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834337.883222966] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834337.883233008] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834337.883247049] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834337.883255924] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834337.883275466] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996252, Target: 1.000000
[INFO] [1703834337.883285216] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036934, Target: 0.037000
[ERROR] [1703834337.883364633] [RTXAgentClientNode]: VLA error: 
[INFO] [1703834337.883464966] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834337.886693216] [move_group_interface]: Planning request accepted
[INFO] [1703834337.918896383] [move_group_interface]: Planning request complete!
[INFO] [1703834337.920066258] [move_group_interface]: time taken to generate plan: 0.00343725 seconds
[INFO] [1703834337.920096049] [RTXAgentClientNode]: move_hand 36 ms
[INFO] [1703834337.925468174] [move_group_interface]: Execute request accepted
[INFO] [1703834337.981339549] [move_group_interface]: Execute request success!
[INFO] [1703834337.982226883] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834337.982323633] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.239535, Y: 0.080254, Z: 0.253599, RPY --  R: 0.049613, P: 0.901187, Y: 0.051008, Q -- W: 0.899895, X: 0.011219, Y: 0.435794, Z: 0.012150
[INFO] [1703834337.982393049] [RTXAgentClientNode]: Target (locobot_base_footprint frame) XYZ -- X: 0.239535, Y: 0.100254, Z: 0.233599, RPY --  R: 0.049613, P: 0.901187, Y: 0.051008, Q -- W: 0.899344, X: 0.033423, Y: 0.434655, Z: 0.033747
[INFO] [1703834337.982530466] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834337.983096716] [move_group_interface]: Planning request accepted
[INFO] [1703834343.489285927] [move_group_interface]: Planning request aborted
[ERROR] [1703834343.489692427] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703834343.490140760] [RTXAgentClientNode]: move_arm  took 5507 ms
[ERROR] [1703834343.490222552] [RTXAgentClientNode]: Planning failed!
[INFO] [1703834343.490691010] [RTXAgentClientNode]: Action took 5955 ms
[INFO] [1703834345.490862386] [RTXAgentClientNode]: *** Iteration 5
[INFO] [1703834345.491376053] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834345.794792970] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834345.794831595] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834345.794844845] [RTXAgentClientNode]: Action server available after 303 ms
[ERROR] [1703834345.794875136] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834345.794880720] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834345.794887428] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834345.794892178] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834345.794896136] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834345.794900761] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834345.794918886] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996252, Target: 1.000000
[INFO] [1703834345.794924136] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036934, Target: 0.037000
[ERROR] [1703834345.794940678] [RTXAgentClientNode]: VLA error: 
[INFO] [1703834345.794997136] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834345.801585345] [move_group_interface]: Planning request accepted
[INFO] [1703834345.838567345] [move_group_interface]: Planning request complete!
[INFO] [1703834345.839424386] [move_group_interface]: time taken to generate plan: 0.0125279 seconds
[INFO] [1703834345.839449261] [RTXAgentClientNode]: move_hand 44 ms
[INFO] [1703834345.840089636] [move_group_interface]: Execute request accepted
[INFO] [1703834345.894582803] [move_group_interface]: Execute request success!
[INFO] [1703834345.895093261] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834345.895246886] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.239534, Y: 0.080254, Z: 0.253599, RPY --  R: 0.049614, P: 0.901186, Y: 0.051009, Q -- W: 0.899895, X: 0.011219, Y: 0.435793, Z: 0.012151
[INFO] [1703834345.895305345] [RTXAgentClientNode]: Target (locobot_base_footprint frame) XYZ -- X: 0.239534, Y: 0.100254, Z: 0.233599, RPY --  R: 0.049614, P: 0.901186, Y: 0.051009, Q -- W: 0.899344, X: 0.033424, Y: 0.434655, Z: 0.033748
[INFO] [1703834345.895968803] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834345.896353053] [move_group_interface]: Planning request accepted
[INFO] [1703834351.420432625] [move_group_interface]: Planning request complete!
[INFO] [1703834351.420829375] [move_group_interface]: time taken to generate plan: 5.01946 seconds
[INFO] [1703834351.421113458] [RTXAgentClientNode]: move_arm  took 5525 ms
[INFO] [1703834351.425503583] [move_group_interface]: Execute request accepted
[INFO] [1703834372.558014010] [move_group_interface]: Execute request success!
[INFO] [1703834372.558237968] [RTXAgentClientNode]: Action took 27066 ms
[INFO] [1703834374.558868094] [RTXAgentClientNode]: *** Iteration 6
[INFO] [1703834374.560968344] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834374.893008428] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834374.893171553] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834374.893849719] [RTXAgentClientNode]: Action server available after 334 ms
[ERROR] [1703834374.893940969] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834374.894128553] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834374.894159386] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834374.894205344] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703834374.894474136] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834374.894628553] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834374.894655303] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834374.909496553] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996252, Target: 1.000000
[INFO] [1703834374.909525761] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036934, Target: 0.037000
[INFO] [1703834374.909627969] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834374.910090303] [move_group_interface]: Planning request accepted
[INFO] [1703834374.941751136] [move_group_interface]: Planning request complete!
[INFO] [1703834374.942769553] [move_group_interface]: time taken to generate plan: 0.00189917 seconds
[INFO] [1703834374.942806428] [RTXAgentClientNode]: move_hand 48 ms
[INFO] [1703834374.943211803] [move_group_interface]: Execute request accepted
[INFO] [1703834374.994680636] [move_group_interface]: Execute request success!
[INFO] [1703834374.995033178] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834374.995170803] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.106153, Y: -0.338420, Z: 0.308480, RPY --  R: -2.624778, P: -0.130585, Y: -0.888848, Q -- W: -0.203105, X: 0.878190, Y: -0.399724, Z: 0.166585
[INFO] [1703834374.995185011] [RTXAgentClientNode]: Target (locobot_base_footprint frame) XYZ -- X: 0.106153, Y: -0.318420, Z: 0.288480, RPY --  R: -2.624778, P: -0.130585, Y: -0.888848, Q -- W: 0.257346, X: -0.863854, Y: -0.429831, Z: -0.052680
[INFO] [1703834374.995250094] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834374.995929511] [move_group_interface]: Planning request accepted
[INFO] [1703834387.596041044] [move_group_interface]: Planning request aborted
[ERROR] [1703834387.597107794] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703834387.597147086] [RTXAgentClientNode]: move_arm  took 12602 ms
[ERROR] [1703834387.597167211] [RTXAgentClientNode]: Planning failed!
[INFO] [1703834387.597174461] [RTXAgentClientNode]: Action took 13038 ms
[INFO] [1703834389.597247295] [RTXAgentClientNode]: *** Iteration 7
[INFO] [1703834389.597606004] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834389.799681129] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834389.799702754] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834389.799758337] [RTXAgentClientNode]: Action server available after 202 ms
[ERROR] [1703834389.799779087] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834389.799784920] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834389.799790795] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834389.799796712] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834389.799800962] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834389.799801629] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703834389.799817379] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834389.799835212] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996252, Target: 1.000000
[INFO] [1703834389.799840754] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036934, Target: 0.037000
[INFO] [1703834389.799910337] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834389.802154587] [move_group_interface]: Planning request accepted
[INFO] [1703834389.861082295] [move_group_interface]: Planning request complete!
[INFO] [1703834389.862147254] [move_group_interface]: time taken to generate plan: 0.0265316 seconds
[INFO] [1703834389.862216045] [RTXAgentClientNode]: move_hand 62 ms
[INFO] [1703834389.862474045] [move_group_interface]: Execute request accepted
[INFO] [1703834389.915196670] [move_group_interface]: Execute request success!
[INFO] [1703834389.916166462] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834389.916293837] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.106150, Y: -0.338420, Z: 0.308480, RPY --  R: -2.624787, P: -0.130588, Y: -0.888851, Q -- W: -0.203101, X: 0.878191, Y: -0.399726, Z: 0.166585
[INFO] [1703834389.916321462] [RTXAgentClientNode]: Target (locobot_base_footprint frame) XYZ -- X: 0.106150, Y: -0.318420, Z: 0.288480, RPY --  R: -2.624787, P: -0.130588, Y: -0.888851, Q -- W: 0.257343, X: -0.863854, Y: -0.429833, Z: -0.052678
[INFO] [1703834389.916520754] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834389.917106212] [move_group_interface]: Planning request accepted
[INFO] [1703834402.464185718] [move_group_interface]: Planning request aborted
[ERROR] [1703834402.464196718] [move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[INFO] [1703834402.464527843] [RTXAgentClientNode]: move_arm  took 12548 ms
[ERROR] [1703834402.464617551] [RTXAgentClientNode]: Planning failed!
[INFO] [1703834402.464670885] [RTXAgentClientNode]: Action took 12867 ms
[INFO] [1703834404.464803719] [RTXAgentClientNode]: *** Iteration 8
[INFO] [1703834404.465822636] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834404.824174428] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834404.824447178] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834404.824485803] [RTXAgentClientNode]: Action server available after 359 ms
[ERROR] [1703834404.824528928] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834404.824540219] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834404.824560928] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834404.824571219] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834404.824576094] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703834404.824604053] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834404.824735678] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834404.824762594] [RTXAgentClientNode]: HAND OPENNESS -- Current: 0.996252, Target: 1.000000
[INFO] [1703834404.824774261] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.036934, Target: 0.037000
[INFO] [1703834404.824912803] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834404.826234344] [move_group_interface]: Planning request accepted
[INFO] [1703834404.871668219] [move_group_interface]: Planning request complete!
[INFO] [1703834404.875202553] [move_group_interface]: time taken to generate plan: 0.0161729 seconds
[INFO] [1703834404.875343344] [RTXAgentClientNode]: move_hand 50 ms
[INFO] [1703834404.878843511] [move_group_interface]: Execute request accepted
[INFO] [1703834404.946843428] [move_group_interface]: Execute request success!
[INFO] [1703834404.947443136] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834404.947491928] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.106150, Y: -0.338420, Z: 0.308480, RPY --  R: -2.624787, P: -0.130588, Y: -0.888851, Q -- W: -0.203101, X: 0.878191, Y: -0.399726, Z: 0.166585
[INFO] [1703834404.947562178] [RTXAgentClientNode]: Target (locobot_base_footprint frame) XYZ -- X: 0.106150, Y: -0.318420, Z: 0.288480, RPY --  R: -2.624787, P: -0.130588, Y: -0.888851, Q -- W: 0.257343, X: -0.863854, Y: -0.429833, Z: -0.052678
[INFO] [1703834404.947811969] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834404.948278969] [move_group_interface]: Planning request accepted
^C[INFO] [1703834416.685327502] [rclcpp]: signal_handler(signum=2)
^[[AKilled
root@d3ec6893a1b6:/simply_ws# ros2 run ros2_transformers rt1_demo_app --ros-args -p use_sim_time:=true -p num_iterations:=10
[INFO] [1703834479.071457295] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 4 seconds
[INFO] [1703834479.071516420] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703834479.227002545] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703834479.243188962] [move_group_interface]: Ready to take commands for planning group interbotix_arm.
[INFO] [1703834479.251425253] [move_group_interface]: Ready to take commands for planning group interbotix_gripper.
[INFO] [1703834479.508072962] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.18 seconds
[INFO] [1703834479.508129379] [moveit_robot_model.robot_model]: Loading robot model 'locobot'...
[WARN] [1703834479.699331337] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1703834479.901757045] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1703834479.902385795] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1703834479.902403295] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1703834479.905032129] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1703834479.905535212] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1703834479.919766462] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1703834479.920211420] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1703834479.990962629] [RTXAgentClientNode]: *** Iteration 0
[ERROR] [1703834479.991337295] [RTXAgentClientNode]: Waiting...
[INFO] [1703834482.993684547] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834483.360365214] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834483.360470130] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834483.360529130] [RTXAgentClientNode]: Action server available after 3369 ms
[ERROR] [1703834483.360575505] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834483.360612672] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834483.360633130] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834483.360650422] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834483.360666880] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834483.361011339] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703834483.360808255] [RTXAgentClientNode]: VLA error: 
[INFO] [1703834483.361568130] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[WARN] [1703834483.364777964] [moveit_ros.current_state_monitor]: Unable to update multi-DOF joint 'virtual_joint':Failure to lookup transform between 'world'and 'locobot_base_footprint' with TF exception: "world" passed to lookupTransform argument target_frame does not exist. 
[INFO] [1703834483.383183505] [RTXAgentClientNode]: HAND OPENNESS -- Current: -1.114286, Target: 0.000000
[INFO] [1703834483.383338964] [RTXAgentClientNode]: FINGER JOINT -- Current: -0.000000, Target: 0.019500
[INFO] [1703834483.383431589] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834483.384273297] [move_group_interface]: Planning request accepted
[INFO] [1703834483.436628130] [move_group_interface]: Planning request complete!
[INFO] [1703834483.437322297] [move_group_interface]: time taken to generate plan: 0.0155969 seconds
[INFO] [1703834483.437366880] [RTXAgentClientNode]: move_hand 76 ms
[INFO] [1703834483.438406922] [move_group_interface]: Execute request accepted
[INFO] [1703834501.033586166] [move_group_interface]: Execute request success!
[INFO] [1703834501.034135375] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834501.052404583] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.308721, Y: 0.000327, Z: 0.247229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: -0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703834501.052465458] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.308721, Y: 0.020327, Z: 0.227229, RPY --  R: -0.000000, P: 0.900000, Y: 0.001058, Q -- W: 0.900447, X: 0.000230, Y: 0.434965, Z: 0.000477
[INFO] [1703834501.052552708] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834501.052895291] [move_group_interface]: Planning request accepted
[INFO] [1703834501.233947291] [move_group_interface]: Planning request complete!
[INFO] [1703834501.234378500] [move_group_interface]: time taken to generate plan: 0.0376664 seconds
[INFO] [1703834501.234456541] [RTXAgentClientNode]: move_arm  took 200 ms
[INFO] [1703834501.238777125] [move_group_interface]: Execute request accepted
[INFO] [1703834504.839722835] [move_group_interface]: Execute request success!
[INFO] [1703834504.840697585] [RTXAgentClientNode]: Action took 24847 ms
[INFO] [1703834506.840857252] [RTXAgentClientNode]: *** Iteration 1
[INFO] [1703834506.841793627] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834507.211723002] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834507.211764377] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834507.211774836] [RTXAgentClientNode]: Action server available after 370 ms
[ERROR] [1703834507.211792586] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834507.211795127] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703834507.211811294] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834507.211817586] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834507.211823961] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834507.211828586] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834507.211833752] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834507.211848669] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.002675, Target: 0.000000
[INFO] [1703834507.211854044] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019453, Target: 0.019500
[INFO] [1703834507.211933711] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834507.217306169] [move_group_interface]: Planning request accepted
[INFO] [1703834507.280176503] [move_group_interface]: Planning request complete!
[INFO] [1703834507.280885461] [move_group_interface]: time taken to generate plan: 0.0261853 seconds
[INFO] [1703834507.280919711] [RTXAgentClientNode]: move_hand 69 ms
[INFO] [1703834507.281487961] [move_group_interface]: Execute request accepted
[INFO] [1703834507.346498461] [move_group_interface]: Execute request success!
[INFO] [1703834507.347438461] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834507.347495961] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.291643, Y: 0.020386, Z: 0.248856, RPY --  R: 0.000354, P: 0.900686, Y: 0.000748, Q -- W: 0.900298, X: -0.000003, Y: 0.435274, Z: 0.000259
[INFO] [1703834507.347508419] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.291643, Y: 0.040386, Z: 0.228856, RPY --  R: 0.000354, P: 0.900686, Y: 0.000748, Q -- W: 0.900298, X: 0.000322, Y: 0.435274, Z: 0.000414
[INFO] [1703834507.347577711] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834507.348497961] [move_group_interface]: Planning request accepted
[INFO] [1703834507.446503169] [move_group_interface]: Planning request complete!
[INFO] [1703834507.447547878] [move_group_interface]: time taken to generate plan: 0.0157801 seconds
[INFO] [1703834507.447635586] [RTXAgentClientNode]: move_arm  took 100 ms
[INFO] [1703834507.448616461] [move_group_interface]: Execute request accepted
[INFO] [1703834511.257512588] [move_group_interface]: Execute request success!
[INFO] [1703834511.258389921] [RTXAgentClientNode]: Action took 4417 ms
[INFO] [1703834513.258442714] [RTXAgentClientNode]: *** Iteration 2
[INFO] [1703834513.260742255] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834513.620058881] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834513.620121131] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834513.620140589] [RTXAgentClientNode]: Action server available after 361 ms
[ERROR] [1703834513.620180214] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834513.620187589] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834513.620394714] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834513.620400964] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834513.620404922] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834513.620408964] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834513.620440464] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.002675, Target: 0.000000
[INFO] [1703834513.620447547] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019453, Target: 0.019500
[INFO] [1703834513.620515672] [move_group_interface]: MoveGroup action client/server ready
[ERROR] [1703834513.620294922] [RTXAgentClientNode]: VLA error: 
[INFO] [1703834513.622047339] [move_group_interface]: Planning request accepted
[INFO] [1703834513.661852464] [move_group_interface]: Planning request complete!
[INFO] [1703834513.663869881] [move_group_interface]: time taken to generate plan: 0.0115822 seconds
[INFO] [1703834513.663984672] [RTXAgentClientNode]: move_hand 43 ms
[INFO] [1703834513.664989881] [move_group_interface]: Execute request accepted
[INFO] [1703834513.722149922] [move_group_interface]: Execute request success!
[INFO] [1703834513.722680964] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834513.722742089] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.274509, Y: 0.040432, Z: 0.250504, RPY --  R: 0.001491, P: 0.900849, Y: 0.000350, Q -- W: 0.900262, X: 0.000595, Y: 0.435348, Z: -0.000167
[INFO] [1703834513.722766006] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.274509, Y: 0.060432, Z: 0.230504, RPY --  R: 0.001491, P: 0.900849, Y: 0.000350, Q -- W: 0.900262, X: 0.000747, Y: 0.435347, Z: 0.000482
[INFO] [1703834513.722878089] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834513.723393714] [move_group_interface]: Planning request accepted
[INFO] [1703834513.846967547] [move_group_interface]: Planning request complete!
[INFO] [1703834513.847872922] [move_group_interface]: time taken to generate plan: 0.0194905 seconds
[INFO] [1703834513.848028089] [RTXAgentClientNode]: move_arm  took 125 ms
[INFO] [1703834513.848735297] [move_group_interface]: Execute request accepted
[INFO] [1703834517.405403424] [move_group_interface]: Execute request success!
[INFO] [1703834517.405773674] [RTXAgentClientNode]: Action took 4147 ms
[INFO] [1703834519.405897592] [RTXAgentClientNode]: *** Iteration 3
[INFO] [1703834519.411816258] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834519.711313258] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834519.711360133] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834519.711374217] [RTXAgentClientNode]: Action server available after 305 ms
[ERROR] [1703834519.711415758] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834519.711441467] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834519.711450592] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834519.711457508] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834519.711465133] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834519.711473258] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834519.711500133] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.002675, Target: 0.000000
[INFO] [1703834519.711530342] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019453, Target: 0.019500
[INFO] [1703834519.711639675] [move_group_interface]: MoveGroup action client/server ready
[ERROR] [1703834519.711873217] [RTXAgentClientNode]: VLA error: 
[INFO] [1703834519.740943967] [move_group_interface]: Planning request accepted
[INFO] [1703834519.781212967] [move_group_interface]: Planning request complete!
[INFO] [1703834519.781974800] [move_group_interface]: time taken to generate plan: 0.0122403 seconds
[INFO] [1703834519.782021883] [RTXAgentClientNode]: move_hand 70 ms
[INFO] [1703834519.782472842] [move_group_interface]: Execute request accepted
[INFO] [1703834519.887023592] [move_group_interface]: Execute request success!
[INFO] [1703834519.887075342] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834519.887139092] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.257471, Y: 0.060407, Z: 0.252196, RPY --  R: 0.001392, P: 0.901617, Y: 0.001863, Q -- W: 0.900095, X: 0.000221, Y: 0.435694, Z: 0.000535
[INFO] [1703834519.887150800] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.257471, Y: 0.080407, Z: 0.232196, RPY --  R: 0.001392, P: 0.901617, Y: 0.001863, Q -- W: 0.900094, X: 0.001032, Y: 0.435692, Z: 0.001142
[INFO] [1703834519.887219509] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834519.888158509] [move_group_interface]: Planning request accepted
[INFO] [1703834520.023385759] [move_group_interface]: Planning request complete!
[INFO] [1703834520.023616509] [move_group_interface]: time taken to generate plan: 0.0207078 seconds
[INFO] [1703834520.023651342] [RTXAgentClientNode]: move_arm  took 136 ms
[INFO] [1703834520.024107509] [move_group_interface]: Execute request accepted
[INFO] [1703834523.263102885] [move_group_interface]: Execute request success!
[INFO] [1703834523.263469677] [RTXAgentClientNode]: Action took 3857 ms
[INFO] [1703834525.263620261] [RTXAgentClientNode]: *** Iteration 4
[INFO] [1703834525.275604511] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834525.660141595] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834525.660255678] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834525.660324428] [RTXAgentClientNode]: Action server available after 396 ms
[ERROR] [1703834525.660474970] [RTXAgentClientNode]: VLA error: 
[ERROR] [1703834525.660629886] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834525.660663095] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834525.660734845] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834525.660779345] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834525.660825928] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834525.660866928] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[INFO] [1703834525.660915470] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.002675, Target: 0.000000
[INFO] [1703834525.660963720] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019453, Target: 0.019500
[INFO] [1703834525.661062845] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834525.661972553] [move_group_interface]: Planning request accepted
[INFO] [1703834525.706692428] [move_group_interface]: Planning request complete!
[INFO] [1703834525.708098386] [move_group_interface]: time taken to generate plan: 0.0129906 seconds
[INFO] [1703834525.708162970] [RTXAgentClientNode]: move_hand 47 ms
[INFO] [1703834525.711257345] [move_group_interface]: Execute request accepted
[INFO] [1703834525.814562053] [move_group_interface]: Execute request success!
[INFO] [1703834525.814594011] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834525.814638803] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.240466, Y: 0.080274, Z: 0.253888, RPY --  R: 0.006421, P: 0.902461, Y: 0.005866, Q -- W: 0.899907, X: 0.001610, Y: 0.436078, Z: 0.001240
[INFO] [1703834525.814651053] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.240466, Y: 0.100274, Z: 0.233888, RPY --  R: 0.006421, P: 0.902461, Y: 0.005866, Q -- W: 0.899898, X: 0.004168, Y: 0.436061, Z: 0.004040
[INFO] [1703834525.814762428] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834525.815868845] [move_group_interface]: Planning request accepted
[INFO] [1703834525.936226595] [move_group_interface]: Planning request complete!
[INFO] [1703834525.936618636] [move_group_interface]: time taken to generate plan: 0.0170239 seconds
[INFO] [1703834525.936651761] [RTXAgentClientNode]: move_arm  took 122 ms
[INFO] [1703834525.937685220] [move_group_interface]: Execute request accepted
[INFO] [1703834529.179603430] [move_group_interface]: Execute request success!
[INFO] [1703834529.181966138] [RTXAgentClientNode]: Action took 3918 ms
[INFO] [1703834531.180709916] [RTXAgentClientNode]: *** Iteration 5
[INFO] [1703834531.181742541] [RTXAgentClientNode]: Goal accepted by server, waiting for result
[ERROR] [1703834531.507900750] [RTXAgentClientNode]: Feedback callback
[INFO] [1703834531.508022500] [RTXAgentClientNode]: Feedback: 0.000000 0.020000 -0.020000
[INFO] [1703834531.508051041] [RTXAgentClientNode]: Action server available after 327 ms
[ERROR] [1703834531.508073000] [RTXAgentClientNode]: move_hand began
[ERROR] [1703834531.508084875] [RTXAgentClientNode]: finger_joint_min_: 0.019500
[ERROR] [1703834531.508090458] [RTXAgentClientNode]: finger_joint_max_: 0.037000
[ERROR] [1703834531.508094583] [RTXAgentClientNode]: left_finger_joint_: left_finger
[ERROR] [1703834531.508098583] [RTXAgentClientNode]: right_finger_joint_: right_finger
[ERROR] [1703834531.508103375] [RTXAgentClientNode]: move_hand_group_: interbotix_gripper
[ERROR] [1703834531.508109875] [RTXAgentClientNode]: VLA error: 
[INFO] [1703834531.508117208] [RTXAgentClientNode]: HAND OPENNESS -- Current: -0.002675, Target: 0.000000
[INFO] [1703834531.508359416] [RTXAgentClientNode]: FINGER JOINT -- Current: 0.019453, Target: 0.019500
[INFO] [1703834531.508461375] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834531.661035542] [move_group_interface]: Planning request accepted
[INFO] [1703834531.703178708] [move_group_interface]: Planning request complete!
[INFO] [1703834531.703192208] [move_group_interface]: time taken to generate plan: 0.0121489 seconds
[INFO] [1703834531.703351292] [RTXAgentClientNode]: move_hand 195 ms
[INFO] [1703834531.704845667] [move_group_interface]: Execute request accepted
[INFO] [1703834531.758110750] [move_group_interface]: Execute request success!
[INFO] [1703834531.758157875] [RTXAgentClientNode]: VLA WORLD VECTOR in move_arm: 0.000000 0.020000 -0.020000
[INFO] [1703834531.758246917] [RTXAgentClientNode]: Current (world frame) XYZ -- X: 0.223426, Y: 0.099933, Z: 0.255449, RPY --  R: 0.017905, P: 0.902802, Y: 0.018963, Q -- W: 0.899797, X: 0.003919, Y: 0.436266, Z: 0.004627
[INFO] [1703834531.758260708] [RTXAgentClientNode]: Target (world frame) XYZ -- X: 0.223426, Y: 0.119933, Z: 0.235449, RPY --  R: 0.017905, P: 0.902802, Y: 0.018963, Q -- W: 0.899723, X: 0.012191, Y: 0.436113, Z: 0.012437
[INFO] [1703834531.758394542] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1703834531.758633542] [move_group_interface]: Planning request accepted
^C[INFO] [1703834534.435643793] [rclcpp]: signal_handler(signum=2)

