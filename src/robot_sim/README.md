# ros_gz_project_template
A template project integrating ROS 2 and Gazebo simulator.

## Included packages

* `model_description` - holds the sdf description of the simulated system and any other assets.

* `gazebo` - holds gazebo specific code and configurations. Namely this is where systems end up.

* `application` - holds ros2 specific code and configurations.

* `bringup` - holds launch files and high level utilities.


## Install

```bash
export GZ_VERSION=harmonic
    
sudo apt install python3-vcstool python3-colcon-common-extensions git wget
```

## Usage

1. Install dependencies

    ```bash
    cd ~/template_ws
    source /opt/ros/$ROS_DISTRO/setup.bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro <ROS_DISTRO>
    ```

1. Build the project

    ```bash
    colcon build --cmake-args -DBUILD_TESTING=ON
    ```

1. Source the workspace

    ```bash
    . ~/template_ws/install/setup.sh
    ```

1. Launch the simulation

    ```bash
    ros2 launch bringup diff_drive.launch.py
    ```

For a more detailed guide on using this template see [documentation](https://gazebosim.org/docs/latest/ros_gz_project_template_guide).
