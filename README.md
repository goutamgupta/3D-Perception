# 3D-Perception
Udacity Robotics Nanodegree - Perception Project 

Environment Setup
It is highly recommended to complete all three RoboND-Perception-Exercises before starting any work on this project..

For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly.
If you do not have an active ROS workspace, you can create one by:

$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
Now that you have a workspace, clone or download the project repository into the src directory of your workspace:

$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Perception-Project.git
Important Note: If you have the Kinematics Pick and Place project in the same ROS Workspace as this project, please remove the gazebo_grasp_plugin directory from this project.

Next install missing dependencies using rosdep install:

$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
Build the project:

$ cd ~/catkin_ws
$ catkin_make
Add the following line to your .bashrc file:

export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/models:$GAZEBO_MODEL_PATH
This allows Gazebo to find custom models used in this project.

If you haven’t already, the following line can be added to your .bashrc to auto-source all new terminals:

source ~/catkin_ws/devel/setup.bash
You have now successfully built the project and are ready to explore it further!

Project Demo
Once your environment is setup correctly and built the project, you can launch the project demo from a sourced terminal by:

$ cd ~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/scripts
$ chmod u+x pr2_safe_spawner.sh
$ ./pr2_safe_spawner.sh
Once Gazebo is up and running, make sure you see the following objects in the gazebo world:

Robot
Table arrangement
Three target objects on the table
Dropboxes on either sides of the robot

On the RViz window, you should be able to see PR2 along with a preliminary collision map and a welcome message.

If there seems to be a problem with either gazebo or RViz, consider closing the demo by pressing ctrl+c in the terminal you used to launch the demo and restarting it.

If everything worked out fine, you can now begin the demo.

Just click on the Next button on the bottom left corner of the RViz window to proceed from one state to another.

Notice how a neat little status message is displayed about the simulator state on top of the robot in RViz window. This status message changes as we move through the different stages of simulation.

The robot starts in an idle state, with both arms extended forward.
Clicking on the Next button will add two dropboxes as collision elements.

Next, the robot raises its arms to enter the so-called "Ready State". This allows the robot to have an unobstructed vision in front of it, evident by a growing collision map.

Further, the robot rotates in-place, first clockwise then anti-clockwise, finally settling at the center "Ready State". This maneuver allows the robot to expand the collision map and include the side tables as well.

Next the robot identifies three tabletop objects (marked by insertion of green object meshes) and starts planning pick and place trajectories.

From here on, just sit back, relax and watch as the robot swiftly accomplishes the pick and place task by depositing all three items.
This signifies the completion of one pick and place operation cycle.

Note that the planning framework creates a new plan with different trajectory points every time, even though the three objects are spawned at the same location.

Play around with this demo and don't forget to have fun!

Important Note: The robot is a bit moody at times and might leave objects on the table or fling them across the room.

Creating the perception pipeline:

Start out by creating a ROS node just like you did in the exercises. In the pr2_robot/scripts/ directory you'll find a file called project_template.py, where you can move over all your code from Exercise-3 (or start from scratch if you like).

First, you'll need to change your subscriber (or create a new subscriber) to subscribe to the camera data (point cloud) topic /pr2/world/points.

To make sure that everything worked fine, try publishing the same data on your own topic and view it in RViz. To refresh your memory on how to do this, follow the tutorial in the Publish Your Point Cloud section.

Filtering
Once you have your camera data, start out by applying various filters you have looked at so far. Keep in mind you'll have to tweak the parameters to accommodate this new environment. Also, this new dataset contains noise! To clean it up, the first filter you should apply is the statistical outlier filter.

Note, the statistical outlier filter in python-pcl was broken for a time and was only fixed after the exercises came out, so if you're getting an error when running the statistical outlier filter that looks like this:

Error: TypeError: __cinit__() takes exactly 1 positional argument (0 given)
You'll need to git pull the RoboND-Perception-Exercises repo to get the latest updates, then re-install python-pcl:

$ cd ~/RoboND-Perception-Exercises/python-pcl
$ python setup.py build
$ sudo python setup.py install
For a reminder on the filters covered in Exercise-1 have a look here

Table Segmentation
Next, perform RANSAC plane fitting to segment the table in the scene. Much like you did in a previous lesson, to separate the objects from the table.


Clustering
Use the Euclidean Clustering technique described here to separate the objects into distinct clusters, thus completing the segmentation process.

Object Recognition
For this project, as you already saw in the previous section, you have a variety of different objects to identify. Essentially, there are three different worlds or scenarios that you are going to work with where each scenario has different items on the table in front of the robot. These worlds are located in the /pr2_robot/worlds/ folder, namely the test_*.world files.

By default, you start with the test1.world but you can modify that in the pick_place_project.launch file in the /pr2_robot/launch/ folder:

  <!--Launch a gazebo world-->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <!--TODO:Change the world name to load different tabletop setup-->
    <arg name="world_name" value="$(find pr2_robot)/worlds/test1.world"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="paused" value="false"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>
  
  
  You can now complete the object recognition steps you performed in Exercise-3 including:

Generate a training set of features for the objects in your pick lists (see the pick_list_*.yaml files in /pr2_robot/config/). Each pick list corresponds to a world and thus indicates what items will be present in that scenario. To generate the training set, you will have to modify the models list in the capture_features.py script and run it as you did for Exercise-3:

if __name__ == '__main__':
  rospy.init_node('capture_node')

  # Modify following list with items from pick_list_*.yaml
  models = [\
     'beer',
     'bowl',
     'create',
     'disk_part',
     'hammer',
     'plastic_cup',
     'soda_can']
Train your SVM with features from the new models.
Add your object recognition code to your perception pipeline.
Test with the actual project scene to see if your recognition code is successful.
To test with the project, first run:

$ roslaunch pr2_robot pick_place_project.launch
and then,

$ rosrun pr2_robot project_template.py
You should arrive at a result similar to what you got in Exercise-3 but this time for new objects in a new environment!


Output .yaml files
For a passing submission of this project, all you need to do is implement your perception pipeline in a new environment and correctly identify the majority of objects in three different scenarios. In each scenario, you'll be faced with the same robot environment and a different collection of objects on the table. In each case you'll be provided with a "pick list" of objects that you're looking for.

These pick lists exist as yaml files under /pr2_robot/config and are loaded to the ros parameter server with the following line in your project launch file:

<rosparam command="load" file="$(find pr2_robot)/config/{PICK_LIST_NAME}.yaml"/>
If you open up the project launch file called pick_place_project.launch in pr2_robot/launch, you'll find the default pick list is set to pick_list_1.yaml, which looks like this:

object_list:
  - name: biscuits
    group: green
  - name: soap
    group: green
  - name: soap2
    group: red

This list dictates the order in which objects are to be collected, along with each object's group, which determines which dropbox the object goes into.

You can retrieve the list from the parameter server and parse it in the following manner. Since the header of the pick list file was object_list, that is the parameter name under which is loaded. To read this parameter, you can use:

# get parameters
object_list_param = rospy.get_param('/object_list')
The object_list_param now holds the pick list and can be parsed to obtain object names and associated group.

object_name = object_list_param[i]['name']
object_group = object_list_param[i]['group']
where i = index used to traverse the list.

For each item in the list you must identify the associated object in the scene and calculate its centroid. To calculate an object's centroid (average position of all the points in the object cloud) you can use a handy pcl method to convert the cloud to an array. First though, recall that in your perception pipeline, you created a list of detected objects (of message type DetectedObject) like this:

# Add the detected object to the list of detected objects.
do = DetectedObject()
do.label = label
do.cloud = ros_cluster
detected_objects.append(do)
For each item in the list, you'll need to compare the label with the pick list and provide the centroid. You can grab the labels, access the (x, y, z) coordinates of each point and compute the centroid like this:

labels = []
centroids = [] # to be list of tuples (x, y, z)
for object in objects:
    labels.append(object.label)
    points_arr = ros_to_pcl(object.cloud).to_array()
    centroids.append(np.mean(points_arr, axis=0)[:3])
WARNING: ROS messages expect native Python data types but having computed centroids as above your list centroids will be of type numpy.float64. To recast to native Python float type you can use np.asscalar(), for example:

>>> import numpy as np
>>> x=np.mean([1.27,2.2,3.99])
>>> print(x)
2.48666666667
>>> type(x)
<class 'numpy.float64'>
>>> type(np.asscalar(x))
<class 'float'>
Now that you have the labels and centroids for target objects in the pick list, you can create the necessary ROS messages to send to the pick_place_routine service.

Note: creating these messages successfully and outputting them to .yaml file format is all you are required to do to complete this project. After that, if you're interested in a challenge you can work on sending these messages and completing the pick and place operations.

The pick and place operation is implemented as a request-response system, where you must write a ros_client to extend a request to the pr2_pick_place_server. Have a look at PickPlace.srv in pr2_robot/srv. This script defines the format of the service message:

# request
std_msgs/Int32 test_scene_num
std_msgs/String object_name
std_msgs/String arm_name
geometry_msgs/Pose pick_pose
geometry_msgs/Pose place_pose
---
# response
bool success

The request your ros_client sends to the pr2_pick_place_server must adhere to the above format and contain:

Name	Message Type	Description	Valid Values
test_scene_num	std_msgs/Int32	The test scene you are working with	1,2,3
object_name	std_msgs/String	Name of the object, obtained from the pick-list	-
arm_name	std_msgs/String	Name of the arm	right, left
pick_pose	geometry_msgs/Pose	Calculated Pose of recognized object's centroid	-
place_pose	geometry_msgs/Pose	Object placement Pose	-
You can obtain the test_scene_num by referring to the world file name, loaded in your pick_place_project.launch file. The line in your launch file looks like this:

<arg name="world_name" value="$(find pr2_robot)/worlds/test1.world"/>
Indicating in this case that the test_scene_num value is 1.

But be careful, the message type you need to send for this variable is not a simple integer but a ROS message of type std_msgs/Int32.

To get more information on this message type. In a terminal type in the following command:

$ rosmsg info std_msgs/Int32
and you'll get:

int32 data
Which means that this message type simply contains a data field of int32 type.

Meaning you cannot simply assign a number to std_msgs/Int32. instead, you must first import this message type into your node:

from std_msgs.msg import Int32
Next, initialize the test_scene_num variable:

test_scene_num = Int32()
And then populate the appropriate data field

test_scene_num.data = 1
Next, the object_name is directly obtained by reading off of the pick list. But again, this is not a simple string but a std_msgs/String type ros message. Just like before, you can investigate the contents of this message type by:

$ rosmsg info std_msgs/String
And find that it contains a single data element:

string data
So in the script for your ROS node import the String message type and populate it with the appropriate value:

# import the message type
from std_msgs.msg import String

# Initialize a variable
object_name = String()

# Populate the data field
object_name.data = object_list_param[i]['name']

Based on the group associated with each object (that you extracted from the pick list .yaml file), decide which arm of the robot should be used.

Since the green box is located on the right side of the robot, select the right arm for objects with green group and left arm for objects with red group. Like you did with object_name, initialize the appropriate message type and populate it with the name of the arm.

Previously you wrote code to calculate the centroid of an identified object, this centroid will now be passed as the pick_pose variable.

Since the message type for pick_pose is geometry_msgs/Pose, just like previous ROS message types, investigate it's contents, import the message type, initialize and then populate the fields with appropriate data.

$ rosmsg info geometry_msgs/Pose
# import message type
from geometry_msgs.msg import Pose

# initialize an empty pose message
pick_pose = Pose()

# TODO: fill in appropriate fields
Now the final argument to be passed for a pick and place request is place_pose. You can retrieve the place pose from the dropbox.yaml.

Just like the pick_list.yaml file, dropbox.yaml is also loaded into the parameter server via the pick_place_project.launch by this line of code:

<rosparam command="load" file="$(find pr2_robot)/config/dropbox.yaml"/>
The dropbox.yaml file looks like this:

dropbox:
  - name: left
    group: red
    position: [0,0.71,0.605]
  - name: right
    group: green
    position: [0,-0.71,0.605]

The position parameters give you the (x, y, z) positions of the centers of the bottom of each drop box.

Similar to what you did to parse the pick list .yaml file, parse dropbox.yaml and retrieve the position parameters. Then for each pick and place operation, populate the place_pose positions with those values.

And that's it! You now have all the messages you need and you're ready to create a .yaml output file. The way it will actually work within your code is that you'll iterate over each item in the pick list, see whether you found it in your perception analysis, then if you did, populate the pick_pose message with the centroid. Since you'll do this with each object one at a time, a convenient way to save the messages for each object is in a list of dictionaries.

We've provided a make_yaml_dict() helper function to help you convert messages to dictionaries. It looks like this:

def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict
This function takes advantage of further helper code in the rospy_message_converter package to convert ROS messages to dictionary format. With each iteration over the pick list, you can create a dictionary with the above function and then generate a list of dictionaries containing all your ROS service request messages. So for example, your for loop might look like this:

dict_list = []
for i in range(0, len(object_list_param)):
    # Populate various ROS messages
    yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
    dict_list.append(yaml_dict)
After finishing iterations, you can send your list of dictionaries to a .yaml file using the yaml Python package and another helper function we provide called send_to_yaml() which looks like this:

def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

  

