#!/usr/bin/env python

import rospy

from sensor_msgs.msg import PointCloud2
from pcl_helper import *

class PointCloudSubscriber:
    """ Subscriber to the Point Cloud """
    def __init__(self):
        """ Subscribe to the pr2 world points """
        self.subscriber = rospy.Subscriber("/camera/depth/color/points",PointCloud2, self.callback, queue_size=1)
        self.pcl_data = None
        
            
    def callback(self,ros_msg):
        """ Callback function for the topic """

        # Convert ROS msg to PCL data
        self.pcl_data = ros_to_pcl(ros_msg) 

        # Statistical Outlier Filtering
        #self.pcl_data=self.statistical(self.pcl_data)

        # Voxel Grid Downsampling
        self.pcl_data=self.voxel_grid(self.pcl_data)

        # PassThrough Filter
        self.pcl_data=self.passthrough(self.pcl_data)

        # RANSAC Plane Segmentation
        pcl_data_table, pcl_data_objects = self.segmenter(self.pcl_data)
                       
        # Convert PCL data table to ROS messages
        ros_data_table = pcl_to_ros(pcl_data_objects)
               
        # Publish ROS data table messages
        pcl_table_pub.publish(ros_data_table)
        
        # Add the table to the collidable objects. It stores the previous point cloud
        # ir order to publish this values as the robot rotate around the scene
        #self.collidable_to_pub_list.append(pcl_data_table)
        #for collidable_to_pub in self.collidable_to_pub_list:
        #    ros_collidable_to_pub = pcl_to_ros(collidable_to_pub)
        #    collidable_pub.publish(ros_collidable_to_pub)
        
        # Euclidean Clustering
        cluster_indices, pcl_data_objects_clustered,pcl_data_objects_xyz = self.cluster(pcl_data_objects)

        # Convert PCL data object clustered to ROS messages
        ros_data_objects_clustered = pcl_to_ros(pcl_data_objects_clustered)
        
        # Publish ROS clustered object messages
        pcl_objects_pub.publish(ros_data_objects_clustered)

        # Classify the clusters! (loop through each detected cluster one at a time)
        #detected_objects_labels,detected_objects = self.classifier(pcl_data_objects,cluster_indices,pcl_data_objects_xyz)
        
        #rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

        # Publish the list of detected objects        
        #detected_object_pub.publish(detected_objects)

        # Clear the octomap to start with a fresh one.
        #self.clear_octomap()           
        
    def statistical(self,pcl_data):
        statistical_outlier_filter = pcl_data.make_statistical_outlier_filter()
        
        statistical_outlier_filter.set_mean_k(10)
        statistical_outlier_filter.set_std_dev_mul_thresh(0.1)
        pcl_data = statistical_outlier_filter.filter()
        return pcl_data
        
    def voxel_grid(self,pcl_data):
        voxel_grid_filter = pcl_data.make_voxel_grid_filter()
        voxel_grid_filter.set_leaf_size(0.005, 0.005, 0.005)
        pcl_data = voxel_grid_filter.filter()
        return pcl_data        
        
    def passthrough(self,pcl_data):
        passthrough_filter = pcl_data.make_passthrough_filter()
        filter_axis = 'z'
        passthrough_filter.set_filter_field_name(filter_axis)
        axis_min = 0.1
        axis_max = 0.9
        passthrough_filter.set_filter_limits(axis_min, axis_max)
        pcl_data = passthrough_filter.filter()
        return pcl_data
    
    def passthrough_y(self,pcl_data):
        passthrough_filter = pcl_data.make_passthrough_filter()
        filter_axis = 'y'
        passthrough_filter.set_filter_field_name(filter_axis)
        axis_min = -0.4
        axis_max = 0.4
        passthrough_filter.set_filter_limits(axis_min, axis_max)
        pcl_data = passthrough_filter.filter()
        return pcl_data

    def segmenter(self,pcl_data):
        segmenter = pcl_data.make_segmenter()    
        segmenter.set_model_type(pcl.SACMODEL_PLANE)
        segmenter.set_method_type(pcl.SAC_RANSAC)    
        segmenter.set_distance_threshold(0.01)
        inliers, coefficients = segmenter.segment()
        # Extract inliers and outliers
        pcl_data_objects = pcl_data.extract(inliers, negative=True)
        pcl_data_table = pcl_data.extract(inliers, negative=False)        
        return pcl_data_table, pcl_data_objects 
   
    def cluster(self,pcl_data_objects):        
        pcl_data_objects_xyz = XYZRGB_to_XYZ(pcl_data_objects)
        tree = pcl_data_objects_xyz.make_kdtree()
        EuclideanClusterExtraction = pcl_data_objects_xyz.make_EuclideanClusterExtraction()
        EuclideanClusterExtraction.set_ClusterTolerance(0.05)
        EuclideanClusterExtraction.set_MinClusterSize(10)
        EuclideanClusterExtraction.set_MaxClusterSize(2000)
        # Search the k-d tree for clusters
        EuclideanClusterExtraction.set_SearchMethod(tree)
        # Extract indices for each of the discovered clusters
        cluster_indices = EuclideanClusterExtraction.Extract()   

        ## Create Cluster-Mask Point Cloud to visualize each cluster separately
        cluster_color = get_color_list(len(cluster_indices))

        color_cluster_point_list = []
        for j, indices in enumerate(cluster_indices):
            for i, indice in enumerate(indices):
                color_cluster_point_list.append([pcl_data_objects_xyz[indice][0],
                                                 pcl_data_objects_xyz[indice][1],
                                                 pcl_data_objects_xyz[indice][2],
                                                 rgb_to_float(cluster_color[j])])
    
        pcl_data_objects_clustered = pcl.PointCloud_PointXYZRGB()
        pcl_data_objects_clustered.from_list(color_cluster_point_list)
        
        return cluster_indices,pcl_data_objects_clustered,pcl_data_objects_xyz
        
    def classifier(self,pcl_data_objects,cluster_indices,pcl_data_objects_xyz):
        detected_objects = []
        detected_objects_labels = []
        ## Classify the clusters! (loop through each detected cluster one at a time)
        for index, pts_list in enumerate(cluster_indices):
            # Grab the points for the cluster
            pcl_data_single_object = pcl_data_objects.extract(pts_list)
            ros_data_single_object = pcl_to_ros(pcl_data_single_object)
            # Compute the associated feature vector
            chists = compute_color_histograms(ros_data_single_object,using_hsv=True)
            normals = get_normals(ros_data_single_object)
            nhists = compute_normal_histograms(normals)
            feature = np.concatenate((chists, nhists))
            # Make the prediction
            prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
            label = encoder.inverse_transform(prediction)[0]
            detected_objects_labels.append(label)
            # Publish a label into RViz
            label_pos = list(pcl_data_objects_xyz[pts_list[0]])
            label_pos[2] += .4
            object_markers_pub.publish(make_label(label,label_pos, index))
            # Add the detected object to the list of detected objects.
            do = DetectedObject()
            do.label = label
            do.cloud = pcl_data_single_object
            detected_objects.append(do)        
        return detected_objects_labels,detected_objects

    def clear_octomap(self):     
        pass
        #rospy.wait_for_service('/clear_octomap')
        #try:
        #    clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        #    clear_octomap()
        #    print ("Response: OK" )     
        #except rospy.ServiceException, e:
        #    print "Service clear_octomap call failed: %s"%e
            
        #    # function to load parameters and request PickPlace service
            
if __name__ == '__main__':

    print("Hello Perception!")

    rospy.init_node("perception_pipeline",anonymous=True)

    # Create Subscribers
    PointCloudSubscriber = PointCloudSubscriber()

    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table",PointCloud2, queue_size=1)
    #object_markers_pub = rospy.Publisher("/object_markers",Marker,queue_size=1)
    #detected_object_pub = rospy.Publisher("/detected_objects",DetectedObjectsArray,queue_size=1)
    #collidable_pub = rospy.Publisher("/pr2/3d_map/points",PointCloud2, queue_size=1)
    #world_joint_pub = rospy.Publisher("/pr2/world_joint_controller/command",Float64,queue_size=1)

    # Load Model From disk
    #model = pickle.load(open('model.sav', 'rb'))
    #clf = model['classifier']
    #encoder = LabelEncoder()
    #encoder.classes_ = model['classes']
    #scaler = model['scaler']

    # Initialize color_list
    #get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()



