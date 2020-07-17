
This is a repo which demonstrates how to interface with a YuMi robot using [Robot Operating System 2](https://www.ros.org/) (ROS) and the [abb_librws](https://github.com/ros-industrial/abb_librws) library.

This is a full copy of the full write-up that is also available on my blog [here](http://sebastiangrans.github.io/Controlling-the-grippers-on-an-ABB-Yumi-using-abb_librws)

I've tried to make the demo as bare bone as possible and consists of a single `.cpp` file.

## Background

Very few people work directly with industrial robots, and even fewer work with [Robot Operating System](https://www.ros.org/) (ROS). For that reason, the community is very small and documentation is scares which makes it can difficult to even get started. 

I am currently in that position myself, in particular with controlling the [ABB IRB 14000](https://new.abb.com/products/robotics/industrial-robots/irb-14000-yumi) robot, also known as the YuMi.

Luckily for me though, my research group recently had some master students working with exactly this task. As you probably know, while working on a master thesis, documentation is rarely your number one priority. Hence I will take that responsibility. 

I will try to continously write tutorials on how to interface with the ABB YuMi robot, and possibly in the future, also some Kuka robots. 

And before we get to it: Mayor shoutout goes to Mariunil and markubjo who has been doing a lot of the heavy lifting. You can check out there code [here](https://github.com/yumi-crew/yumi/). They based their development on the [first ROS package](https://github.com/OrebroUniversity/yumi) for the YuMi robot which was made by people at Örebro University in Sweden. 


## Preparing the robot

To interface with the robot and its grippers we will use a library called [librws](https://github.com/ros-industrial/abb_librws). It gives us an intermediate API to the [Robot Web Services](https://developercenter.robotstudio.com/api/rwsApi/) so that we don't have to deal with all that RESTful stuff.

**Installing the StateMachine Add-In**  
librws relies on the [StateMachine Add-In](https://robotapps.robotstudio.com/#/viewApp/c163de01-792e-4892-a290-37dbe050b6e1) to be running on the robot. This can be installed using RobotStudio. This was already done on the robot I am working on, so I can't give any details on how that is done, but the manual seems to be comprehensive.

**Installing the SmartGripper Add-In**  
Same with this one, it was already installed. Check out robot studio and you can probably figure it out.

## Preparations

I assume that you already have ROS2 installed and have some knowledge. If not, begin by reading the [ROS2](https://index.ros.org/doc/ros2/) installation and tutorial pages. FYI: I am running ROS2 Eloquent on Ubuntu 18.04.

For librws, we need to install [POCO](https://github.com/pocoproject/poco). 

First some dependencies:
```
apt-get install openssl libssl-dev
sudo apt-get -y update && sudo apt-get -y install git g++ make cmake libssl-dev
```
Then download and build: 
```bash
git clone -b master https://github.com/pocoproject/poco.git
cd poco
mkdir cmake-build
cd cmake-build
cmake ..
cmake --build . --config Release
``` 
and finally installation:
```
sudo cmake --build . --target install
```

Then we can proceed by creating a new ROS workspace so that we have a clean slate.
```bash
mkdir -p ~/test_ws/src
cd ~/test_ws/src
```

We can now proceed by cloning the librws library. 
```bash
git clone -b master https://github.com/ros-industrial/abb_librws.git
```

I will show two examples, one that is a minimalistic barebone executable, and another that a ROS2 Node with some slightly more refined features.

Either you can follow along here, or simply download the demo package by cloning this GitHub repo.TODO!

If you're following along, you can begin by creating a ROS2 package, which I call `sg`: 

```bash
cd ~/test_ws/
ros2 pkg create --build-type ament_cmake sg
```

(*Note:* If you get the error `ros2: command not found`, you have forgotten to source the ROS intallation. Do this first by running `source /opt/ros/eloquent/setup.bash`.)

## Minimal example:
Let's begin with the minimal example by creating the file `minimal_example.cpp` in the folder `~/test_ws/src/sg/src/`. The content of which is the following:

```cpp
#include <abb_librws/rws_state_machine_interface.h>
int main() {
    // Set up an interface to the robot using abb_librws
    // Make sure that the IP matches that of your robot! 
    abb::rws::RWSStateMachineInterface rws_state_machine_interface_  = 
        abb::rws::RWSStateMachineInterface("192.168.125.1");
    // Before we can use the gripper, it needs to be calibrated. 
    rws_state_machine_interface_.services().sg().rightCalibrate();
    // Close the grip.
    rws_state_machine_interface_.services().sg().rightGripIn();
    usleep(2*1000000); // Wait 2 seconds.
    // Open the grip.
    rws_state_machine_interface_.services().sg().rightGripOut();
    return 0;
}
```

In order to compile this, we need to modify the auto-generated `package.xml` and `CMakeList.txt` files. To the first one, you need to add the following lines:

```xml
<depend>rclcpp</depend>
<depend>abb_librws</depend>
```

(**Note:** I do not know why `rclcpp` is required, but without it you will get a CMake Error.)

And in `CMakeList.txt`: 

```cmake
# Required packages minimal_example
find_package(rclcpp REQUIRED)
find_package(abb_librws REQUIRED)

## minimal_example
add_executable(minimal_example src/minimal_example.cpp)
ament_target_dependencies(minimal_example
                          abb_librws
)
# Installs the executable in ~/test_ws/install/sg/lib/
install(TARGETS 
          minimal_example
          DESTINATION lib/${PROJECT_NAME}
)
```

### Compiling
We can now compile this into a ROS2 executable!

```bash
cd ~/test_ws/ 
colcon build
```
And you should be greeted with the following output:
```
Starting >>> abb_librws
Finished <<< abb_librws [0.22s]                       
Starting >>> sg
Finished <<< sg [0.18s]      

Summary: 2 packages finished [0.50s]
```

In order for the CLI Tools to find the package you just compiled, you need to source the local installation by running:
```bash
source install/local_setup.bash
```

### Running

First we need to do some hardware preparations.

1. Connect your PC to the robot by hooking up an Ethernet cable to its "Service" port.
2. On the FlexPendant, make sure that no tasks are running by pressing the stop button (■).
3. In the Production Window, press "PP to Main". This resets the program pointer and makes sure that the tasks run from the beginning. 
4. Set the robot into "Auto mode".
5. Run the tasks by pressing the play button (▶).


If everything works out as it should, you should see the following in the logs:
```
T_ROB_R->[Main]: Starting StateMachine loop
T_ROB_L->[Main]: Starting StateMachine loop
T_ROB_R->[Main]: Idling...
T_ROB_L->[Main]: Idling...
```

On the computer, you can now run your executable:
```
ros2 run sg minimal_example
```

The gripper should now move, and you should see the following lines in the logs on the FlexPendant:
```
T_ROB_R->[SmartGripper]: Received command 2 (Calibrate Gripper)
T_ROB_R->[SmartGripper]: Received command 5 (Grip outwards)
T_ROB_R->[SmartGripper]: Received command 4 (Grip inwards)
```

Success! 

## A proper example
Typically we run our executables as ROS Nodes which communicates with each other. In this example, we will create a ROS2 Node that listens on the `/grip` topic.

**Note:** It would be more appropriate to implement this as a [service](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Service-And-Client/) or an [action](https://index.ros.org/doc/ros2/Tutorials/Actions/), but for simplicity we will simply use a topic subscriber.  


In `~/test_ws/src` create `sg_controller.cpp`.

The entire code can be seen in the spoiler: 
```cpp
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp> // This is deprecated in newer versions of ROS.
#include <abb_librws/rws_state_machine_interface.h>

class SmartGripper : public rclcpp::Node {
public:

    // Constructor
    SmartGripper(std::string node_name, const std::string &robot_ip) :
        Node(node_name), 
        ip_(robot_ip) {}

    bool Init() {

        // This object is our way to interface with the robot. 
        rws_state_machine_interface_  = 
        std::make_shared<abb::rws::RWSStateMachineInterface>(ip_);

        // We check that we can connect.
        auto runtime_info = rws_state_machine_interface_->collectRuntimeInfo();
        RCLCPP_INFO(this->get_logger(), "Connecting to Robot...");
        if (!runtime_info.rws_connected) {
            RCLCPP_ERROR(this->get_logger(), "Connection failed.");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "OK!");

        // And we make sure that the StateMachine RAPID task is running! 
        RCLCPP_INFO(this->get_logger(), "Checking for StateMachine...");
        if (!rws_state_machine_interface_->isRAPIDRunning().isTrue()) {
            RCLCPP_ERROR(this->get_logger(), 
                "StateMachine is not running on the robot.");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "OK!");

        // Before we can use the gripper, it needs to be calibrated. 
        // Either we can do it programmatically, or we can also calibrate 
        // on the FlexPendant while we also start the StateMachine.
        RCLCPP_INFO(this->get_logger(), "Calibrating SmartGrippers...");
        if (!rws_state_machine_interface_->services().sg().rightCalibrate()) {
            RCLCPP_ERROR(this->get_logger(), "Could not calibrate!");  
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "OK!");


        // We set up a subscription on the topic /grip [std_msgs/msg/Bool]
        // With a callback to grip_callback(std_msgs::msgs::Bool::SharedPtr msg)
        grip_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                        "grip", 10,
                        std::bind(
                            &SmartGripper::grip_callback, 
                            this, 
                            std::placeholders::_1
                        )
                    );

        RCLCPP_INFO(this->get_logger(), "Init complete!");
        return true;
    }

private:
    std::string ip_;
    std::shared_ptr<abb::rws::RWSStateMachineInterface> 
        rws_state_machine_interface_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grip_sub_;
    bool grip_callback(std_msgs::msg::Bool::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received grip request: %d", msg->data);
        bool retval;
        // All the boilerplate code is just so that we can execute these commands.
        if (msg->data) { 
            retval = rws_state_machine_interface_->services().sg().rightGripIn();
        } else {
            retval = rws_state_machine_interface_->services().sg().rightGripOut();
        }

        if(!retval) {
            RCLCPP_ERROR(this->get_logger(), "Unable to grip."); 
        }   
        return retval;
    }
}; 
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto sg = std::make_shared<SmartGripper>("sg", "192.168.125.1");
    if (sg->Init()){
        RCLCPP_INFO(sg->get_logger(), "Spinning node!");
        rclcpp::spin(sg->get_node_base_interface());
    } else {
        RCLCPP_ERROR(sg->get_logger(), "Could not initiate. Terminating..");
    }
    rclcpp::shutdown();
    return 0;
}
```

If we break it down, it has the essentials from `minimal_example` but is written as a Node object with some additional verbosity features sprinkled in.

* We confirm that we are connected to the robot.
* We confirm that the StateMachine is running on the robot. 
* We confirm that calibration of the gripper is successful.
* We set up a subscription and a callback function to the `/grip` topic.

### Compiling
Just as for the `minimal_example`, we need to add some changes to the `CMakeList.txt`:

```cmake
find_package(rclcpp REQUIRED)
find_package(abb_librws REQUIRED)
find_package(std_msgs REQUIRED)

## sg_controller_node
add_executable(sg_controller_node src/sg_controller.cpp)
ament_target_dependencies(sg_controller_node
                          rclcpp
                          abb_librws
                          std_msgs
)
install(TARGETS 
          sg_controller_node
          DESTINATION lib/${PROJECT_NAME}
)
```
And `package.xml`:
```xml
<depend>rclcpp</depend> 
<depend>std_msgs</depend>
<depend>abb_librws</depend>
```

### Compilation 

Just as for the `minimal_example`, we compile it by running:
```bash
cd ~/test_ws/ 
colcon build
```
Then again we need to source the local installation:
```bash 
source install/local_setup.bash
```

(**Note:** While building this, I get the error `/usr/bin/ld: warning: libPocoFoundation.so.50, needed by /opt/ros/eloquent/lib/librmw_implementation.so, may conflict with libPocoFoundation.so.71` which I haven't been able to figure out how to solve. The example runs fine nonetheless.
)

### Running

Like in the `minimal_example`, you need to do some preparations on the robot. 

Then you can run the node:
```
ros2 run sg sg_controller_node
```
and you should be greeted with:

```
[INFO] [sg]: Connecting to Robot...
[INFO] [sg]: OK!
[INFO] [sg]: Checking for StateMachine...
[INFO] [sg]: OK!
[INFO] [sg]: Calibrating SmartGrippers...
[INFO] [sg]: OK!
[INFO] [sg]: Init complete!
[INFO] [sg]: Spinning node!
``` 
And on the robot you should see:
```
T_ROB_R->[SmartGripper]: Received command 2 (Calibrate gripper)
```
Then, in order to actually move the gripper, you can open another terminal and publish a message to the `/grip` topic like so:
```
ros2 topic pub --once /grip std_msgs/msg/Bool "{data: false}"
```
Setting the `data` field to `false` opens the grippers, while `true` closes them. 

Hopefully this is what you see on your robot:

<center>
<img src="{{ site.baseurl }}/images/20200717-Grippers/gripping-loop.gif" alt="Controlling the YuMi SmartGrippers using librws" style="display: block;"/>
</center>





## Troubleshooting 

### Error: `Package 'sg' not found`
If you get this error when trying to run e.g. `ros2 run sg minimal_example` it means that you forgot to source the local installation after building. 

I.e. you need to first run: `source install/local_setup.bash`. 

### Nothing happens on the robot 
* Did you remember to reset the Program Pointer in the Production Window on the FlexPendant? 
* Did you remember to set the robot in "Auto Mode"?

### Other possible issues
[Here](https://github.com/ros-industrial/abb_librws/issues/70) is an issue on GitHub related to the Smart Grippers. If you are having issues, you might find help there. 

