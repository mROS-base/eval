# uros-freertos

- Middleware: micro-ROS
- RTOS: FreeRTOS (freertos)
- Board: STM32 Nucleo F767ZI (nucleo_f767zi)
- Apps: [mROS-base/freertos_apps](https://github.com/mROS-base/freertos_apps/tree/eval/galactic-fix-upd-nucleo-f767zi)
  - forked from [micro-ROS/freertos_apps PR #97](https://github.com/micro-ROS/freertos_apps/pull/97)
  - added some applications to compare with mros2 env

## Procedure (a.k.a memo)

It's about the same as the official tutorial on micro-ROS website: https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/

### Building the evaluation app

- [same] Installing ROS 2 and the micro-ROS build system
  ```
  # Source the ROS 2 installation
  source /opt/ros/$ROS_DISTRO/setup.bash

  # Create a workspace and download the micro-ROS tools
  cd <dir_of_this_repo>/uros-freertos
  mkdir -p microros_ws/src
  cd microros_ws/src
  git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
  cd ..

  # Update dependencies using rosdep
  sudo apt update && rosdep update
  rosdep install --from-path src --ignore-src -y

  # Install pip
  sudo apt-get install python3-pip

  # Build micro-ROS tools and source them
  colcon build
  source install/local_setup.bash
  ```
- Creating a new firmware workspace
  - specify Platform to `nucleo_f767zi`
    ```
    ros2 run micro_ros_setup create_firmware_ws.sh freertos nucleo_f767zi
    ```
  - change remote url and branch of `freertos_apps`
    ```
    cd firmware/microros_apps
    git remote set-url origin https://github.com/mROS-base/freertos_apps
    git pull
    git checkout eval/galactic-fix-upd-nucleo-f767zi
    ```
- Configuring the firmware
  - need to set [APP] according to the target (e.g., `eval_string`)
  - for serial connection
    ```
    ros2 run micro_ros_setup configure_firmware.sh [APP] -t serial
    ```
  - for udp connection ([IP] is about the Agent)
    ```
    ros2 run micro_ros_setup configure_firmware.sh [APP] -t udp -i [IP] -p 8888
    ```
- Building and Flashing the firmware
  ```
  ros2 run micro_ros_setup build_firmware.sh
  ros2 run micro_ros_setup flash_firmware.sh
  ```
  - adding `-f` to build_firmware.sh is useful to fast build

### Executing the evaluation app

- [same] Creating the micro-ROS agent
  ```
  ros2 run micro_ros_setup create_agent_ws.sh
  ros2 run micro_ros_setup build_agent.sh
  source install/local_setup.bash
  ```
- Running the micro-ROS app
  - for serial connection
    ```
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-STM<snip.>
    ```
  - for udp connection
    ```
    ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888
    ```
  - 
