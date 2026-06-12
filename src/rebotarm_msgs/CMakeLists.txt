cmake_minimum_required(VERSION 3.8)
project(rebotarm_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection2D.msg"
  "msg/Detection2DArray.msg"
  "msg/GraspCandidate.msg"
  "msg/GraspCandidateArray.msg"
  "msg/GraspPlan.msg"
  "msg/ExecutionState.msg"
  "msg/TaskStatus.msg"
  "msg/JointMotorCmd.msg"
  "msg/JointMotorState.msg"
  "msg/ArmStatus.msg"
  "srv/PlanGraspForLabel.srv"
  "srv/SetMode.srv"
  "srv/SetZero.srv"
  "srv/SetTeachRecordPath.srv"
  "srv/MoveToPoseIK.srv"
  "srv/ExecutePose.srv"
  "srv/SetGripper.srv"
  "srv/GraspGripper.srv"
  "action/ExecuteGrasp.action"
  "action/MoveRelative.action"
  "action/MoveToPose.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
