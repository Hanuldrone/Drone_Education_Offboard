# Drone_education_cmd

## Build Package
    mkdir -p ~/ws_education/src
    cd ~/ws_education/src
    catkin init 
    git clone https://github.com/Hanuldrone/Drone_Education_Offboard.git
    cd ..
    catkin build

## Run Offboard
### terminal 1
    cd ~/PX4-Autopilot/launch
    roslaunch mavros_posix_sitl.launch 
### terminal 2
    cd ~/Download/
    ./QGroundControl.AppImage
### terminal 3
    cd ~/ws_education
    source ./devel/setup.bash
    rosrun offboard offboard_node


## Adjust Safety Radius & Gain
    - param_id
    [0]: Safety radius
    [1]: Gain Value Default
    [2]: Gain Value Y    

### Command
    rosservice call /offboard_param "param_id: ''
    value:
      integer: 0
      real: 0.0" 
