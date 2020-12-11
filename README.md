# RViz Panel

This repository demonstrates the method for writing a rviz dockable panel.

Note that to generate the `ui*.h` file you will need to use the `add_library` or `add_executable` function for it to appear.

![image](resource/img.png)

## Steps

Note that plugin.xml class name should be unique for ROS to locate it. Otherwise, only the first that registered is found.

1. [Create Package](###Create-Package)
1. [Create Necessary Folders and Files](###Folder-Structure)
1. [Create UI file with Qt Designer](https://github.com/BruceChanJianLe/ros-rqt-plugin#ui-file)
1. [Update package.xml (update export tag)](###Packagexml)
1. [Update CMakeLists.txt](###CMake)
1. [Compile to create header file from UI file (catkin_make)](###UI-Header)
1. [Create and update header file (inside of include/<package_name>)](###Header-File)
1. [Create and update source file (inside of src)](###Source-File)
1. [Compile (catkin_make)]

### Create Package

Create your personal ROS RViz panel/plugin with the following command:
```bash
catkin_create_pkg rviz-panel roscpp rospy pluginlib rviz std_msgs
```
Please choose carefully your packages, here std_msgs is no needed to actually build the panel.  

### Folder-Structure
```bash
├── CMakeLists.txt
├── include
│   └── rviz-panel
│       └── rviz_panel.hpp  # Header File
├── package.xml
├── resource
│   └── simple_panel.ui     # UI File
├── rviz_plugin.xml         # Declare plugin
└── src
    └── rviz_panel.cpp      # Source File
```

### Package.xml
Do not forget to add export tag correctly.
```xml
<?xml version="1.0"?>
<package format="2">
  <name>rviz-panel</name>
  <version>0.0.0</version>
  <description>The rviz-panel package</description>

  <author email="jianle001@e.ntu.edu.sg">Bruce Chan Jian Le</author>
  <maintainer email="jianle001@e.ntu.edu.sg">Bruce Chan Jian Le</maintainer>
  <license>MIT</license>
  <url type="website">https://github.com/BruceChanJianLe/rviz-panel</url>


  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>rviz</build_depend>
  <build_depend>pluginlib</build_depend>
  <build_depend>std_msgs</build_depend>

  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>rviz</build_export_depend>
  <build_export_depend>pluginlib</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>rviz</exec_depend>
  <exec_depend>pluginlib</exec_depend>
  <exec_depend>std_msgs</exec_depend>


  <export>
    <rviz plugin="${prefix}/rviz_plugin.xml"/>
  </export>

</package>
```

### CMake
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(rviz-panel)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
  pluginlib
  # Other dependecies
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES rviz-panel
  # CATKIN_DEPENDS roscpp rospy rviz
  # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

# Avoid keyword definition to avaid conflicts with boost or xapian etc
# e.g. http://muddyazian.blogspot.de/2012/04/getting-qt-app-working-with-boost-using.html
add_definitions(-DQT_NO_KEYWORDS)

# Define source file
set(${PROJECT_NAME}_SRCS
  src/rviz_panel.cpp
)

# Define header file
set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/rviz_panel.hpp
)

# Define ui file
set(${PROJECT_NAME}_UIS
  resource/simple_panel.ui
)

# Create header from ui file (uic)
if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Generate header for ui with rviz_QT_VERSION: ${rviz_QT_VERSION}")
    qt4_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
    qt4_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
else()
    message(STATUS "Generate header for ui with rviz_QT_VERSION: ${rviz_QT_VERSION}")
    qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
    qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
endif()

## Add library is needed in order to generate the header file from ui file.
add_library(simple_panel
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_UIS_H}
  ${${PROJECT_NAME}_MOCS} 
)

target_link_libraries(simple_panel
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)
```

### UI Header

In order to generate the header file from the UI file, you will need to use the add_library function to invoke CMakeLists.txt to create the header file. The header file will be inside of the build directory. To use the header file in vscode you may have to use `#include <package_name/ui_header.h` but to compile it, you will need to change it to `#include <ui_header.h>`.

To include the ui header file, you need to include this cmake binary directory which is the build file.
```cmake
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}       # To include UI header file
)
``` 

## Reference

- RViz Panel Example (repository) [link](https://github.com/ros-visualization/visualization_tutorials/tree/8284284b3894a7c7c9298e2018f040894daa4779/rviz_plugin_tutorials)
- RViz Panel from ui file [link](https://answers.ros.org/question/241811/build-rviz-plugin-from-ui-file/)
- Simple RViz Plugin (repository) [link](https://gitlab.com/InstitutMaupertuis/simple_rviz_plugin)
- Question on uic [link](https://www.qtcentre.org/threads/35960-How-to-convert-ui-file-to-cpp-file-in-QtCreator)