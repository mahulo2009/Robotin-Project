#!/usr/bin/env python
import rospy
import pickle

from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2

from features import compute_color_histograms
from features import compute_normal_histograms
from features import get_normals

from pcl_helper import *


def capture_sample():
    return rospy.wait_for_message('/pcl_objects', PointCloud2)   

if __name__ == '__main__':
    rospy.init_node('capture_node')

    models = [\
       'yellow ball',
       'candle',
       'toten']

    labeled_features = []

    for model_name in models:
        print("Capturing ", model_name )
        input("Press enter to continue.")
        for i in range (20):
            print("Pos ", i )
            input("Press enter to continue.")

            for i in range(5):
                print("Capturing ", model_name, "pose ", i )
                #input("Press Enter to continue...")

                # make five attempts to get a valid a point cloud then give up
                sample_was_good = False
                try_count = 0
                while not sample_was_good and try_count < 5:
                    sample_cloud = capture_sample()
                    sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()

                    # Check for invalid clouds.
                    if sample_cloud_arr.shape[0] == 0:
                        print('Invalid cloud detected')
                        try_count += 1
                    else:
                        sample_was_good = True

                # Extract histogram features
                chists = compute_color_histograms(sample_cloud, using_hsv=False)
                normals = get_normals(sample_cloud)
                nhists = compute_normal_histograms(normals)
                feature = np.concatenate((chists, nhists))
                labeled_features.append([feature, model_name])
            
        print(model_name, "procced" )

    pickle.dump(labeled_features, open('training_set.sav', 'wb'))