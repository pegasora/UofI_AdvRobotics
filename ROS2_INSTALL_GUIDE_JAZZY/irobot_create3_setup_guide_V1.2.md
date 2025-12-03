![irobot education](irobot-tm-education_elongated.png)

# irobot create3 setup guide

follow this document to install ros2 and setup a ros2 connection with the irobot create3. it is recommended that you follow the installation process outlined below, and refer to the documentation provided by ros2 and irobot for detailed instructions. refer to this document for quick reference as needed.

### how to follow this guide 
there are three main steps in this guide:

1. install ros2 
2. setup/configure the irobot create3
3. install irobot create3 dependencies/libraries

the steps may be performed in parallel or individually, and are fairly independent of each other. verifying installation for steps two and three require having ros2 installed first, so that is the recommended starting point here.

### extra resources for learning ros2
if this is your first time using ros2, you will likely want to read through/follow the beginner tutorials provided by ros2 which introduce the `ros2` command line interface, package system, and `colcon` build system. these tutorials are linked in the [documentation links](#documentation-links) below and are most helpful after installing ros2. ros2 is **not** an intuitive system at first glance with several quirks (cough _sourcing_ cough _workspaces_ cough) that may make it seem like black magic, but the tutorials and documentation go a long way towards pulling back to the curtain, so to speak.


## table of contents
1. [documentation/tutorial links](#documentation-links)

2. [setup guide](#setup-guide)
	1. [ros2 installation guide](#ros2-installation)
	2. [irobot create3 configuration guide](#irobot-create3-configuration-guide)
	3. [installing ros2 irobot create3 dependencies](#installing-ros2-irobot-create3-dependencies)
	4. [verify ros2 & create3 connection](#verification)


# documentation links
| link | description | 
| :---: | :---: | 
| [ros2: humble hawksbill documentation](https://docs.ros.org/en/humble/index.html) | latest ros2 version (as of fall 2022) | 
| [ros2: humble cli tutorials](https://docs.ros.org/en/humble/tutorials/beginner-cli-tools.html) | introduction to command line tools provided by ros2 for _debugging!_ | 
| [ros2: humble client lib tutorials](https://docs.ros.org/en/humble/tutorials/beginner-client-libraries.html) | introduction to ros2 library creation and build systems. |  
| [ros2: all tutorials](https://docs.ros.org/en/humble/tutorials.html) | tutorials for all experience levels whereas the two above are introductory. |
| [ros2 python api: rclpy](https://docs.ros2.org/latest/api/rclpy/index.html) | python api for using ros2 interfaces and writing custom ros2 nodes. |
| [rclpy examples](https://github.com/ros2/examples) | python and c++ example code for nodes, topics, services, and actions. | 
| [irobot create3 code examples](https://github.com/iroboteducation/create3_examples) | examples of how to control the robot using command topics and actions. **very** helpful. |
| [irobot create3: getting started guide](https://edu.irobot.com/learning-library/create-3-getting-started) | hardware documentation and setup guides for first steps. |
| [irobot create3 documentation](https://iroboteducation.github.io/create3_docs/) | detailed documentation for configuration and debugging. | 
| [irobot create3: apis](https://iroboteducation.github.io/create3_docs/api/ros2/) | ros2 api for create3. |


# setup guide

## ros2 installation
![humble](humble-small.png)

install ros2 'humble' or newer on a linux machine. for installation instructions and latest release go [here](https://www.ros.org/blog/getting-started/). make sure to install the latest *ros2* version, not *ros1*. they are both listed on the linked page. if you like, test your installation by running the `turtlesim` publish/subscribe example at the end of the ros2 installation guide. a good follow-up to this example is following the cli and client library tutorials that are linked in the [documentation links](#documentation-links) above. these tutorials will continue to use `turtlesim` to illustrate the architecture used by ros2 and introduce debugging tools.

if you have followed this guide before and simply want to connect to the create3 as quickly as possible, see the install summary below.

##### summary
this is a summary of [the ros2 installation guide](https://docs.ros.org/en/humble/installation.html) and [the colcon installation guide](https://docs.ros.org/en/humble/tutorials/beginner-client-libraries/colcon-tutorial.html).
although not short, these installation steps will provide all the tools needed to use ros2 on your system.

1. verify locale is utf-8: `locale`
2. verify universe repository enabled on system:
    1. `sudo apt install software-properties-common`
    2. `sudo add-apt-repository universe`

3. setup sources(ros2 apt repository): 
	1. `sudo apt update && sudo apt install curl gnupg lsb-release`
	2. `sudo curl -ssl https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`
	3. `echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $ubuntu_codename) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`

4. _**important:**_ ros2 documentation notes that failure to update may cause catastrophic removal of vital system packages in ubuntu 22.04.

		sudo apt update & sudo apt upgrade

5. install ros2 with: 

		sudo apt install ros-humble-desktop
		
6. run:

		echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

##### install cyclonedds
cyclonedds operates as the ros2 middle-ware(rmw), which manages network communication between ros2 nodes.
1. run:

        sudo apt install ros-humble-rmw-cyclonedds-cpp

2. next, instruct ros2 to use cyclonedds by exporting rmw_implementation to to your system variables:

        echo "export rmw_implementation=rmw_cyclonedds_cpp" >> ~/.bashrc

    * if you would like to verify that this is working, follow the 'run the talker and listener' instructions at the bottom of [this page](http://docs.ros.org.ros.informatik.uni-freiburg.de/en/humble/installation/dds-implementations/working-with-eclipse-cyclonedds.html?highlight=cyclonedds).


##### installing colcon and auto-completion
the steps below are necessary for coding your own ros2 nodes and compiling them with the irobot create3 packages.

1. install colcon to build custom packages:

        sudo apt install python3-colcon-common-extensions

2. setup `colcon_cd`:

        echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
        echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc

3. setup `colcon` auto-completion:

        echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc


##### installation verification
_**to verify:**_ run `ros2 topic list`. you should see something like: 

![topic list output](topic_list.png)

you should **at least see /rosout and /parameter_events**. all other topics listed above are published by the robot, not seeing them may mean that they haven't been received yet(try running the command again) or that the robot isn't connected to your network. if there fewer outputs, follow [the robot configuration guide](#irobot-create3-configuration-guide) to connect the robot to your network and update the firmware. 

if you get the full output above, you have verified ros2 is running and that the node running on the robot is visible to your machine.


## irobot create3 configuration guide
ros2 communicates with the irobot create3 via a 2.4ghz network so we need to configure it to connect to the network we will be using. it will also be important to configure the robot with a unique namespace so that it can operate on the same network as other robots. to do so, follow the written instructions in [this guide](https://edu.irobot.com/create3-setup) and give it a namespace of the pattern `/create3-ap_name` (for example `create3-05f8`) where `ap_name` is the last four digits of the ssid broadcast by the robot. if this doesn't make sense, this should happen in section two, step two of the guide above (or see step 5 below).

###### _**important:**_ 
if you have assigned a namespace to your robot, all actions that you send to the robot **must start with the namespace**. for example, if your robot has the namespace `/create3_05f8` sending the dock command below will **not** work:

	ros2 action send_goal /dock irobot_create_msgs/action/dockservo "{}"

but this will:

	ros2 action send_goal /create3_05f8/dock irobot_create_msgs/action/dockservo "{}"


##### summary
a summary of the configuration in that guide is included below (no firmware update):

1. with robot docked, press and hold buttons on either side of the power button until light turns blue and you hear a beep.
2. connect to `create-xxxx` network from laptop/desktop.
3. open browser and navigate to access point(ap) `192.168.10.1`. this ap can later be reached if you know the ip address of the robot, this can be found by scanning the network with [angry ip](https://angryip.org/), enabling the mac address, mac vendor fetchers, and looking for device from 'irobot' vendor.
4. once ap is reached, navigate to the 'connect' page and use settings to connect to your desired network. 
5. finally, navigate to the application configuration page and change namespace field to `/create3_xxxx` where `xxxx` matches the last four digits of the network ssid broadcast by the robot. in reality, this namespace could be anything you want, but the convention is for easy of use in cs453/553.

###### _**note:**_ 
the steps above don't include firmware update, so if firmware is older than `h0.0` (as of 12/12/2022), update. see irobot documentation at this link: [firmware overview](https://iroboteducation.github.io/create3_docs/releases/overview/). 

## installing ros2 irobot create3 dependencies
there are two steps that must be completed before your ros2 instance on your computer will be capable of communicating with the irobot create3. the first step is to select the ip layer protocol used for transmitting packets from the robot to your machine. the default configuration used by irobot is called cyclonedds, but is not installed by default by ros2. installing and setting up cyclonedds was covered in the first part of this guide under ros2 installation.

the second dependency that must be installed is the library which defines the message types and valid contents accepted by the irobot create3 when enclosed in a cyclonedds packet. to see an example of how a message `type` is formatted, try running `ros2 topic list -t` to see currently observed topics listed, followed by their type.

##### install irobot create3 messages
1. clone repository to `/opt/ros/humble/src` with `sudo git clone -b humble https://github.com/iroboteducation/irobot_create_msgs.git`
2. follow the steps in the verification guide below to make sure that this is used correctly. if you experience problems, try cloning to the `create3_examples_ws/src/` directory that you create in the verification guide.

###### _**note:**_
1. there are significant differences between robot firmware and the create3 messages api exposed by the ros2 package above for certain versions. these differences may break functionality by removing or adding supported topics and actions by changing the name/type of certain interfaces. to avoid headache, make sure your installed firmware matches the version of create3 messages api you have installed. 
2. it is also possible to install the messages package in a ros2 workspace every time you need it. this may help you stay up to date on changes to the package and api, but is not necessary and will add the step of cloning the package to your workflow for each new project.

it is also possible to compare the interfaces exposed/published by the robot with `ros2 <topic/action/service> list -t` to print message types for comparison with the interfaces defined in the create3 messages package. if the interface **name** or **type** is different from the robot, you will likely be unable to use that interface. to view all information about an interface installed in a package on your machine first source the workspace then use the command `ros2 interface show <interface-type-here>` to view argument types and structure.

## verification
the final step is to verify connection between your laptop and the robot. to do this we will install and run the coverage example from [irobot's create3 examples repository](https://github.com/iroboteducation/create3_examples). follow the instructions in the readme of that repository to create and install a new workspace, then follow the directions in `/create3_coverage` to run the example. if successful, your robot will undock and begin driving across the floor until it hits an obstacle.

##### summary
1. _**installation**_, do once. run the following in any directory you wish, but `/home/<user-name>/documents` aka `~` is recommended.

	``` bash
	mkdir -p create3_examples_ws/src
	cd create3_examples_ws/src
	git clone -b humble https://github.com/iroboteducation/create3_examples.git
	cd ..
	rosdep install --from-path src --ignore-src -yi
	colcon build 
	```

2. _**initialization**_, must be done every time a terminal/session is opened for working on this project. do not run the commands below in the same terminal you executed the build command in. open a new terminal session and run: `source ~/create3_examples_ws/install/local_setup.sh`

	1. in first terminal run: `ros2 run create3_coverage create3_coverage`. this creates a new ros2 node on your computer from which you can issue commands to the robot.

		* _**note:**_ if you have set a namespace for your robot, you will want to run the line above as `ros2 run create3_coverage create3_coverage --ros-args -r __ns:=/<create3-namespace>`
	
	2. in a second terminal: `ros2 action send_goal robot_namespace_here/coverage create3_examples_msgs/action/coverage "{explore_duration:{sec: 500, nanosec: 0}, max_runtime:{sec: 1000,nanosec: 0}}"`. this command publishes a command for the robot to fulfill. if the task hangs, with no response from the robot to signal that the command as been received or accepted, verify that you are publishing to the correct namespace such as `create3-0f58`.
