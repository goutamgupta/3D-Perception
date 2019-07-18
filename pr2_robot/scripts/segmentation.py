#!/usr/bin/env python

# Import modules
from pcl_helper import *

# TODO: Define functions as required


# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
    #convert ROS msg to PCL data 
    cloud = ros_to_pcl(pcl_msg)
 
    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE =0.003
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
    filename = 'statoutlier.pcd'
    pcl.save(cloud_filtered,filename)


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
    white_cloud =XYZRGB_to_XYZ(extracted_outliers) # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(50000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()


    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a color corresponding to each segmented object in scene
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
    pcl.save(cluster_cloud, filename)


 
    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects =pcl_to_ros(extracted_outliers) 
    ros_cloud_table = pcl_to_ros(extracted_inliers)
    ros_cloud_cluster = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cloud_cluster)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)


    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1) 
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)

    # TODO: Create Publishers for clusters
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1) 
    




    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

