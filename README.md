# PyTrees Ros Viewer

A Qt-JS application for visualisation of executing/log-replayed behaviour trees
in a ROS2 ecosystem. It currently services as the gui visualisation tool for
[ROS2 PyTrees](https://github.com/splintered-reality/py_trees_ros#pytrees-ros-ecosystem),
but has been designed so that it could be easily extended for use with
other behaviour tree implementations. Create an issue here if you'd like to work
through details on this.  

Feature:

* Render the Behaviour Tree Graph
* Collapsible Subtrees
* Zoom and Scale Contents to Fit
* Timeline Rewind & Resume
* Blackboard (key-value storage) View
* Activity View

Quickstart instructions:

```
# Only released in rosdistro's dashing and later
sudo apt install ros-<rosdistro>-py-trees-ros-tutorials
sudo apt install ros-<rosdistro>-py-trees-ros-viewer

# In a first shell
py-trees-tree-viewer
# In a second shell
ros2 run py_trees_ros_tutorials tutorial-eight-dynamic-application-loading
# Click 'Scan' on the qt robot dashboard interface
# Wear a colander, I am.
```

![Example Video](images/trees.png?raw=true "Rendering Trees w/ Timeline")
