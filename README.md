# RViz Panel

This repository demonstrates the method for writing a rviz dockable panel.

Note that to generate the `ui*.h` file you will need to use the `add_library` or `add_executable` function for it to appear.

![image](resource/img.png)

## Steps

Note that plugin.xml class name should be unique for ROS to locate it. Otherwise, only the first that registered is found.

1. [Create Package](###Create-Package)
1. [Create Necessary Folders and Files](###Folder-Structure)
1. [Create UI file with Qt Designer]
1. [Update package.xml (update export tag)]
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


## Reference

- RViz Panel Example (repository) [link](https://github.com/ros-visualization/visualization_tutorials/tree/8284284b3894a7c7c9298e2018f040894daa4779/rviz_plugin_tutorials)
- RViz Panel from ui file [link](https://answers.ros.org/question/241811/build-rviz-plugin-from-ui-file/)
- Simple RViz Plugin (repository) [link](https://gitlab.com/InstitutMaupertuis/simple_rviz_plugin)
- Question on uic [link](https://www.qtcentre.org/threads/35960-How-to-convert-ui-file-to-cpp-file-in-QtCreator)