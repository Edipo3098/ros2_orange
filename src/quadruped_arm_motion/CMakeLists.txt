cmake_minimum_required(VERSION 3.5)
project(quadruped_arm_motion)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(xacro REQUIRED)  # Ensure xacro is found

# Install directories (URDF, Launch, RViz, CONFIG)
install(DIRECTORY launch urdf rviz config meshes xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
