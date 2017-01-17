# FUTROBOT-ROS

ROS port of the POTI-UFRN's soccer team software for the IEEE Very Small Size
League.

**NOTE:** This project is a work in progress, although some of its parts can
be already used.

**NOTE:** Part of the source code and comments is written in brazilian
portuguese, as it was in the original code.

## Background :

TODO

## Features :

TODO

## Installation :

Clone the repository or create a symbolic link into the `src` folder of your
catkin workspace and run `catkin_make`. For example, if you have a catkin
workspace in `~/catkin_ws`, you can do:

    cd ~/catkin_ws/src
    clone git@github.com:Ellon/futrobotros.git
    cd ~/catkin_ws
    catkin_make

## Usage :

Run `roslaunch` with the `start_*` launch files provided in [launch
directory](launch/) to set up the system. Then use `rviz` for visualization
and `rqt_graph` to inspect the system structure.

### To run simulated mode :

    roslaunch futrobotros start_simulated.launch

### To visualize :

The `rviz` configuration file provided in
[rviz/futrobotros.rviz](rviz/futrobotros.rviz) provides you with all the
displays needed to visualize the data being produced by the nodes. Run:

    rviz -d ~/catkin_ws/src/futrobotros/rviz/futrobotros.rviz

to load it when starting `rviz`.

## License :

TODO