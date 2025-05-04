cmake_minimum_required(VERSION 3.5)
project(mpu_pub)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# Install directories (URDF, Launch, RViz, CONFIG)
install(DIRECTORY launch   config 
  DESTINATION share/${PROJECT_NAME}
)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Mpu.msg"
)

ament_package()
