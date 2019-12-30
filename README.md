# PyTrees Ros Viewer

[[About](#about)] [[Features](#features)] [[Quickstart](#quickstart)] [[Usage](#usage)] [[Modding](#modding)]

## About

A Qt-JS application for visualisation of behaviour trees in a ROS2 ecosystem. This
implementation serves as the gui visualisation tool for
[ROS2 PyTrees](https://github.com/splintered-reality/py_trees_ros#pytrees-ros-ecosystem).

## Features

* Tree Visualisation
* Collapsible Subtrees
* Zoom and Scale Contents to Fit
* Timeline Rewind & Resume
* Blackboard View
* Activity View

## Quickstart

```
sudo apt install ros-<rosdistro>-py-trees-ros-tutorials
sudo apt install ros-<rosdistro>-py-trees-ros-viewer

# In a first shell
py-trees-tree-viewer
# In a second shell
ros2 launch py_trees_ros_tutorials tutorial_eight_dynamic_application_laoding_launch.py
# Click 'Scan' on the qt robot dashboard interface
# Wear a colander, I am too.
```

## Usage

Until a snapshot stream is discovered or selected, you'll land at the splash screen which
enumerates the interactive options available.

![Splash](images/splash.png?raw=true "Splash Screen")

The viewer will continuously scan for and update the namespace drop-down
with a list of py_tree instances (specifically, the namespace of their snapshot
stream services). If not yet connected, it will provide the convenience of
automatically making a connection to the first instance it discovers. Beyond that,
it is possible to switch between streaming services via the drop-down. 

![Select](images/select.png?raw=true "Select a Tree Stream")

At a minimum, the stream will send updates to the tree graph when the tree changes
(i.e. when any one of the behaviours modifies it's status). Additional configuration
of the stream can be managed via the checkboxes in the top right of the window - introspect
on the blackboard and/or request more frequent updates (useful when tracking
blackboard changes). 

![Reconfigure](images/reconfigure.png?raw=true "Reconfigure the Stream")

If you have a large tree, collapse sections of it. This is useful in tandem with
the screenshot capability when you wish to highlight an area of interest to communicate
a design problem or report a bug.

![Collapsible Subtrees](images/collapse.png?raw=true "Collapsible Subtrees")

In the same vein, filter the blackboard to those keys expicitly associated (used)
by manually selected behaviours.

![Track Data](images/track.png?raw=true "Track Blackboard Data")

Problems debugging a catastrophe and the final state of the tree doesn't uncover
the root cause, merely the mayhem that resulted? This is a very common situation - use
the timeline! Rewind to a previous state and time travel. Note: the stream configuration
only applies to future messages - you can't change the past! If you are concerned about
being able to debug affectively, make sure you have at least the 'Blackboard Data'
and 'Periodic' checkboxes enabled so that time travelling along the timeline has
sufficient data for analysis. 

![Rewind](images/rewind.png?raw=true "Rewind")

Travelling along the timeline is possible even while online. Simply hit the 'Resume'
button to resume visualisation of the live feed.

![Resume](images/resume.png?raw=true "Resume")

## Modding

The underlying `py_trees_js` library lends itself to being used in various ways. This
application can be used as a baseline reference (since it utilises the underlying
`py_trees_js` library fully) for modifications to suit your own use case. Open an
[issue](https://github.com/splintered-reality/py_trees_ros_viewer/issues) if you have
questions and/or would like assistance.

1) Visualisation for a behaviour trees implementation that is not `py_trees`.

Replicate the hybrid qt-js application here and replace [backend.py](https://github.com/splintered-reality/py_trees_ros_viewer/blob/devel/py_trees_ros_viewer/backend.py).
The data structure eventually passed to the js library is a json dictionary that only
makes use of the most fundamental properties of behaviour trees (e.g. tree graph
structure, behaviour status) and consequently, it should be a matter of merely
wiring connections and converting data.

2) Visualisation for a different middleware architecture

This follows much the same procedure as for the first use case - replace
[backend.py](https://github.com/splintered-reality/py_trees_ros_viewer/blob/devel/py_trees_ros_viewer/backend.py). For example, a ROS1 viewer for py_trees
could be built in this manner.

3) Visualisation in a mobile device

Since the core of the application is a js library, the bridge to moving from a
developer friendly application to a lightweight web application, or a widget embedded
in another web application is much smaller. Again it is merely a matter of
generating the required input channel and data conversions. Refer to the
`py_trees_js` [README](https://github.com/splintered-reality/py_trees_js#usage) for more
details.
