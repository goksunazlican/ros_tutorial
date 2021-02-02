# Simple Ros Package

A single package has 2 nodes and 2 topics created for understanding ros.


## Tutorials

[Ros tutorials](http://wiki.ros.org/ROS/Tutorials) were used in this repository.


## Problem
The publisher node sends a random variable to subscriber, then subscriber performs a calculation and publish the output.
#### -Generator Node(Python):
Generates a random number with 20hz frequency and publishes it to *"values"* topic in [std_msgs::UInt32](http://docs.ros.org/en/api/std_msgs/html/msg/UInt32.html) message format.

#### -Processor Node(C++):
Subscribes to the *"values"* topic. Performs [ n^2 + t_old] operation and publishes the result to *"results"* topic in [std_msgs::String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html) format. 
n is the current value received. t_old is the previous value calculated. (take t_old = 0 for initialization).
### Pre requierements
If you didn't set up yet, you can follow the [ros tutorials](http://wiki.ros.org/ROS/Tutorials) from installing to creating ros workspace and ros package.

## Generator Node: talker.py
You can reach the source code from src/package2/src/talker.py.

First, we need to add the libraries that *rospy*, *String* and *UInt32* from standart messages and *randint *from random.
```python
import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt32
from random import randint
```

The publisher has the ***talker*** function definition. Talker function creates a random value between 0-500, then it publishes the value in 20 hz rate. Uses ***unsigned*(i)** to convert the integer to uint32. Creates publisher node with *'values'* topic and sends *UInt32* type message.
```python
def talker():
    pub = rospy.Publisher('values', UInt32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20) # 20hz
    while not rospy.is_shutdown():
	    num = unsigned(randint(0,500))
        rospy.loginfo(num)
        pub.publish(UInt32(num))
        rate.sleep()
```
Defines the ***unsigned*** function to convert integer to uint32:
```python
def unsigned(i):
  return i&0xFFFFFFFF
```
In main, calls the talker for publishing:
```python
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

## Processor Node: listener.cpp
As in the talker, we should include libraries such as *ros.h* from ros, *String.h* from std_msgs, *UInt32.h* from std_msgs and *sstream*.
```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt32.h>
#include <sstream>
```
Defines the global variables that *t_old* and *result*.
```cpp
uint32_t t_old=0;
uint32_t result;
```
In the processor node, we aim to subscribe the message with ***values*** topic then publish the result of calculation with ***results*** topic. So, we need a subscriber and a publisher in one node. For this reason, we use the *SubscribeAndPublish* class.

We have private variables such as a Ros node: ***n_***, publisher: ***result_pub***, and subscriber: ***process_sub***. Also, there are a public ***constructor*** and a ***callback*** function.

In the constructor, we assign result_pub and process_sub with node functions. result_pub publishes the result with results topic as a String value. process_sub subscribes a uint32 message that comes from the callback function with the values topic.

The callback calculates the result with uint32 input value, then converts the result from uint32 to string and publishes the string message.
```cpp
class SubscribeAndPublish{
  public:
    SubscribeAndPublish(){
     result_pub = n_.advertise<std_msgs::String>("results",10);
     process_sub = n_.subscribe("values", 10, &SubscribeAndPublish::callback, this);
    }
 void callback(const std_msgs::UInt32 msg){
	  result = (msg.data*msg.data)+t_old;
	  t_old = result;

	  std_msgs::String pub_msg;
	  pub_msg.data = std::to_string(result);

	  result_pub.publish(pub_msg);

	  ROS_INFO("Published result is %s",pub_msg.data.c_str());
	  ROS_INFO("Subscriber: I heard: [%i]  result is: %i and t_old is: %i", msg, result, t_old);
 }
private:
	 ros::NodeHandle n_;
	 ros::Publisher result_pub;
	 ros::Subscriber process_sub;
};
```

In the main, we call the ***ros::init*** and create an object of ***SubscribeAndPublish*** class, then call the ***ros::spin*** for running until Ctrl-C command.
```cpp
int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "listener");
  SubscribeAndPublish SAPObject;
  ros::spin();
  return 0;
}
```

Before run the project, we need to do configurations in package.xml and CMakeList.txt files.
## package.xml:
```xml
<?xml version="1.0"?>
<package format="2">
  <name>package2</name>
  <version>0.0.0</version>
  <description>The package2 package</description>
  <maintainer email="nazli@todo.todo">nazli</maintainer>
  <license>BSD</license>
 <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <export>
  </export>
</package>	

```

## CMakeList.txt:
    cmake_minimum_required(VERSION 3.0.2)
    project(package2)
    
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
    )
    
    include_directories(
      include
      ${catkin_INCLUDE_DIRS}
    )
    
    add_executable(listener src/listener.cpp)
    
     target_link_libraries(listener
       ${catkin_LIBRARIES}
     )
    
     catkin_install_python(PROGRAMS
       src/talker.py
       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
     )


Run roscore in another terminal:
 ```bash
      $ roscore
```
So, for run the project first go the workspace folder and run CMake. We use CMake as our build system:
 ```bash
       $ cd ~/catkin_ws2
	  ~/catkin_ws$ catkin_make
	  ~/catkin_ws$ source devel/setup.bash
```
Then run publisher and subscriber nodes in different terminals.
For publisher: rosrun package_name publisher_node_name

```bash
      ~/catkin_ws$ rosrun package2 talker.py
```

For subscriber: rosrun package_name subscriber_node_name
```bash
      ~/catkin_ws$ rosrun package2 listener.cpp
```

Outputs look like:

Talker Output
```markdown
[INFO] [1611929150.263102]: 52
[INFO] [1611929150.313098]: 4
[INFO] [1611929150.362895]: 24
[INFO] [1611929150.413078]: 360
[INFO] [1611929150.462891]: 86
[INFO] [1611929150.513175]: 101
[INFO] [1611929150.563444]: 179
[INFO] [1611929150.612836]: 0

```

Listener Output
```markdown
[ INFO] [1611929150.268580729]: Subscriber: I heard: [52]  result is: 59867248 and t_old is: 59867248
[ INFO] [1611929150.318590742]: Published result is 59867264
[ INFO] [1611929150.318714041]: Subscriber: I heard: [4]  result is: 59867264 and t_old is: 59867264
[ INFO] [1611929150.368350093]: Published result is 59867840
[ INFO] [1611929150.368469136]: Subscriber: I heard: [24]  result is: 59867840 and t_old is: 59867840
[ INFO] [1611929150.418440076]: Published result is 59997440
[ INFO] [1611929150.418560293]: Subscriber: I heard: [360]  result is: 59997440 and t_old is: 59997440
[ INFO] [1611929150.468406560]: Published result is 60004836
[ INFO] [1611929150.468530531]: Subscriber: I heard: [86]  result is: 60004836 and t_old is: 60004836
[ INFO] [1611929150.519380008]: Published result is 60015037
[ INFO] [1611929150.519519270]: Subscriber: I heard: [101]  result is: 60015037 and t_old is: 60015037
[ INFO] [1611929150.568933617]: Published result is 60047078
[ INFO] [1611929150.569081508]: Subscriber: I heard: [179]  result is: 60047078 and t_old is: 60047078
[ INFO] [1611929150.617925912]: Published result is 60047078
[ INFO] [1611929150.618066935]: Subscriber: I heard: [0]  result is: 60047078 and t_old is: 60047078

```


## License
[MIT](https://choosealicense.com/licenses/mit/)
