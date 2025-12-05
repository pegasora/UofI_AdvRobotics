# ROS2 FANUC Driver Info Page

## How to run code 

1. source the ROS2 env 
 
    `source /opt/ros/jazzy/setup.sh`

2. Optionally, add this to the end of your .bashrc file to automagically do this upon opening of the terminal

    `echo "source /opt/ros/jazzy/setup.sh" >> ~/.bashrc`

3. change into the root of the repo 

4. make sure you have the following packages installed

    - pycomm3 (globally or using your venv managment of choise)
    - ros2 (jazzy)

5.  init rosdep (first time only)

    `rosdep init`
    `rosdep update`

    
6. build the workspace with colcon

    `colcon build`

7. source the workspace

    `source install/setup.sh`

    
8. run the code (server node)

    `ros2 launch launch/start.launch.py robot_name:=<robot_name> robot_ip:=<robot_ip>`
    
    
9. run the code (client node, in a separate terminal)

    `source /opt/ros/jazzy/setup.sh`
    `source install/setup.sh`
    `python3 <nameofthefile>.py`

10. Change code ass appropriate in the client side code 

## common errors

### pycomm3 not installed
- if you get an error saying that pycomm3 is not installed, run the following

`sudo apt install python3-pycomm3`
or 
`pip3 install pycomm3 --break-system-packages`
(this will work for this assignment, you will want to do this is a venv at some point)

### no ros2 command found
- if you get an error saying that ros2 is not found/recognized, run the following

`source /opt/ros/jazzy/setup.sh`

You will need to do this every time you open a new terminal, or see the instructions above to add this to your .bashrc file

### errors with sourcing 

- if you want to use some sort of venv or venv managment, you will need to first source that, source ros, colcon build, THEN you can source the install/setup.sh.
- This process can be tricky. 

## Nix 

I will soon have a nix setup posted in this repo with instructiosn for how to use it (post conversation with Dr. Shovic)

