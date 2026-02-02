# Install script for directory: /home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/src/vyomaveera_gz

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/install/simulation_gazebo")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/simulation_gazebo" TYPE PROGRAM FILES
    "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/src/vyomaveera_gz/scripts/forward_kinematics.py"
    "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/src/vyomaveera_gz/scripts/inverse_kinematics.py"
    "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/src/vyomaveera_gz/scripts/forward_kinematics_module.py"
    "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/src/vyomaveera_gz/scripts/inverse_kinematics_module.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation_gazebo" TYPE DIRECTORY FILES
    "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/src/vyomaveera_gz/config"
    "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/src/vyomaveera_gz/launch"
    "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/src/vyomaveera_gz/meshes"
    "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/src/vyomaveera_gz/rviz"
    "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/src/vyomaveera_gz/scripts"
    "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/src/vyomaveera_gz/urdf"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/build/simulation_gazebo/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/simulation_gazebo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/build/simulation_gazebo/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/simulation_gazebo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation_gazebo/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation_gazebo/environment" TYPE FILE FILES "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/build/simulation_gazebo/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation_gazebo/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation_gazebo/environment" TYPE FILE FILES "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/build/simulation_gazebo/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation_gazebo" TYPE FILE FILES "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/build/simulation_gazebo/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation_gazebo" TYPE FILE FILES "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/build/simulation_gazebo/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation_gazebo" TYPE FILE FILES "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/build/simulation_gazebo/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation_gazebo" TYPE FILE FILES "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/build/simulation_gazebo/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation_gazebo" TYPE FILE FILES "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/build/simulation_gazebo/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/build/simulation_gazebo/ament_cmake_index/share/ament_index/resource_index/packages/simulation_gazebo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation_gazebo/cmake" TYPE FILE FILES
    "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/build/simulation_gazebo/ament_cmake_core/simulation_gazeboConfig.cmake"
    "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/build/simulation_gazebo/ament_cmake_core/simulation_gazeboConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation_gazebo" TYPE FILE FILES "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/src/vyomaveera_gz/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/sarvesh/VyomaVeera.V-1/manipulation/vyomaveera_ros_ws/build/simulation_gazebo/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
