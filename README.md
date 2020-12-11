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
1. [Update CMakeLists.txt]
1. [Compile to create header file from UI file (catkin_make)]
1. [Create and update header file (inside of include/<package_name>)]
1. [Create and update source file (inside of src)]
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

## Reference

- RViz Panel Example (repository) [link](https://github.com/ros-visualization/visualization_tutorials/tree/8284284b3894a7c7c9298e2018f040894daa4779/rviz_plugin_tutorials)
- RViz Panel from ui file [link](https://answers.ros.org/question/241811/build-rviz-plugin-from-ui-file/)
- Simple RViz Plugin (repository) [link](https://gitlab.com/InstitutMaupertuis/simple_rviz_plugin)
- Question on uic [link](https://www.qtcentre.org/threads/35960-How-to-convert-ui-file-to-cpp-file-in-QtCreator)