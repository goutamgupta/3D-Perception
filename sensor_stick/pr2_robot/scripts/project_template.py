#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)


    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE =0.03
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
    filename = 'voxel_downsampled.pcd'
    pcl.save(cloud_filtered, filename)


    
    # TODO: Statistical Outlier Filtering
      # Much like the previous filters, we start by creating a filter object:
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()    
      # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(50)
      # Set threshold scale factor
    x = 1.0
      # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)
      # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter() 
    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE =0.003
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
    filename = 'voxel_downsampled.pcd'
    pcl.save(cloud_filtered, filename)

    # TODO: PassThrough Filter along z axis 
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()
    filename = 'pass_through_filtered_z.pcd'
    pcl.save(cloud_filtered, filename)

   # TODO: PassThrough Filter along y axis 
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.5
    axis_max = 0.5
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()
    filename = 'pass_through_filtered_y.pcd'
    pcl.save(cloud_filtered, filename)


    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()


    # TODO: Extract inliers and outliers
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    filename = 'extracted_inliers.pcd'
    pcl.save(extracted_inliers, filename)

    extracted_outliers = cloud_filtered.extract(inliers, negative=True)
    filename = 'extracted_outliers.pcd'
    pcl.save(extracted_outliers, filename)


    # TODO: Euclidean Clustering
    white_cloud =XYZRGB_to_XYZ(extracted_outliers) # Apply function to convert $
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(50000)
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
       for i, indice in enumerate(indices):
               color_cluster_point_list.append([white_cloud[indice][0],
                                        white_cloud[indice][1],
                                        white_cloud[indice][2],
                                         rgb_to_float(cluster_color[j])])

   #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    filename = 'cluster_cloud.pcd'
    pcl.save(cluster_cloud,filename)


    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects =pcl_to_ros(extracted_outliers) 
    ros_cloud_table = pcl_to_ros(extracted_inliers)
    ros_cloud_cluster = pcl_to_ros(cluster_cloud)


    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cloud_cluster)
  
# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster
        pcl_cluster = extracted_outliers.extract(pts_list)
        # TODO: convert the cluster from pcl to ROS using helper function
        ros_pcl_cluster = pcl_to_ros(pcl_cluster)
   
        # Compute the associated feature vector
        chists = compute_color_histograms(ros_pcl_cluster, using_hsv=True)
        normals = get_normals(ros_pcl_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
 
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

  
        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))
 
        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_pcl_cluster
        detected_objects.append(do)
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    
    # Get the pick list labels 
    object_list_param = rospy.get_param('/object_list')
    pick_list_labels = []
    for i in range(len(object_list_param)):
        pick_list_labels.append(object_list_param[i]['name'])

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    #compare if the set of detected object label is same as the set of pick list objects
    if set(detected_objects_labels) ==set(pick_list_labels):
       print ("detected object matched listed objects")
       try:
           pr2_mover(detected_objects)
       except rospy.ROSInterruptException:
           pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()
    dict_list = []
    centroids = []
    
    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_list_param = rospy.get_param('/dropbox')
    # TODO: Parse parameters into individual variables
    test_scene_num.data = 1
    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for i in range(0, len(object_list_param)):
        object_name.data = object_list_param[i]['name']
        for j in range(0, len(object_list)): 
             if object_list[j].label == object_list_param[i]['name']: 
        # TODO: Get the PointCloud for a given object and obtain it's centroid and assign pick_pose
                 points_arr = ros_to_pcl(object_list[j].cloud).to_array()
                 centroids = np.mean(points_arr, axis=0)[:3]
                 pick_pose.position.x = np.asscalar(centroids[0])
                 pick_pose.position.y = np.asscalar(centroids[1])
                 pick_pose.position.z = np.asscalar(centroids[2])
                 print("obtained  pick pose for ",object_list[i].label) 

        # TODO: Create 'place_pose' for the object
        # TODO: Assign the arm to be used for pick_place
        if object_list_param[i]['group'] =='red':
                  arm_name.data = 'left'
                  place_pose.position.x = dropbox_list_param[0]['position'][0]
                  place_pose.position.y = dropbox_list_param[0]['position'][1]
                  place_pose.position.z = dropbox_list_param[0]['position'][2] 

        else: 
                  arm_name.data = 'right'
                  place_pose.position.x = dropbox_list_param[1]['position'][0]
                  place_pose.position.y = dropbox_list_param[1]['position'][1] 
                  place_pose.position.z = dropbox_list_param[1]['position'][2] 


        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name,object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    send_to_yaml('output_1.yaml', dict_list)
    print("completed writing to output file") 

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    object_markers_pub = rospy.Publisher("/objects_markers", Marker, queue_size=1) 
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_world", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1) 
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)


    # TODO: Load Model From disk
    model = pickle.load(open('model_bin32_300pr2_linear.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']


    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()


