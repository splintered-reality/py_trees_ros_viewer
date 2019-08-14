# PyTrees Ros Viewer

A Qt-JS application for visualisation of executing/log-replayed behaviour trees
in a ROS2 ecosystem.

* Render Behaviour Tree Streams
* Collapsible Subtrees
* Zoom and Scale Contents to Fit
* Timeline Rewind & Resume

Quickstart instructions:

```
# Only released in rosdistro's dashing and later
sudo apt install ros-<rosdistro>-py-trees-ros-tutorials
sudo apt install ros-<rosdistro>-py-trees-ros-viewer

# In a first shell
py-trees-tree-viewer
# In a second shell
ros2 run py_trees_ros_tutorials tutorial-eight-dynamic-application-loading
```

![Example Video](images/trees.png?raw=true "Rendering Trees w/ Timeline")
