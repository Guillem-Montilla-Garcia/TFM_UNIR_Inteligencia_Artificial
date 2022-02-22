#!/usr/bin/env python

#Code developed by Guillem Montilla Garcia
#Date: 22/02/2022


###Libraries###
import numpy as np
from scipy import ndimage
import rospy

from sensor_msgs.msg import Range , LaserScan
from subdron_mission_handler.msg import MHVehicleOdom

from std_srvs.srv import Trigger, TriggerResponse
import tf2_ros

#Funtion name: max_intensities_filter
#Inputs:
## intensities_vector_input -> vector that contains the integer intensities associated to each range.
## input_ranges_vector -> vector that contains the float ranges that have an intensity related.
## k_threshold-> input threshold parameter that sets the integer value to binarize the intensities.
## k_opening_size -> input parameter to select how many pixels are in the structure element used on the opening
## k_closing_size -> input parameter to select how many pixels are in the structure element used on the closing
#Output: 
##filtered_range -> the function outputs a float range value obtained after filtering
def max_intensities_filter(intensities_vector_input,input_ranges_vector ,k_threshold,  k_opening_size, k_closing_size):
  #Applying a binary threhsold according to k_threshold parameter.
  binary_vector_input = (intensities_vector_input > k_threshold).astype(int)
  
  #Creating morphological filter structures
  opening_element = np.ones(k_opening_size)
  closing_element = np.ones(k_closing_size)

  #Applying first a closing with the specified structure element to connect detections.
  #After that, it is applied a opening with the specified structure element to eliminate detections with not enought size.
  filtered_vector = ndimage.binary_opening(ndimage.binary_closing( binary_vector_input,closing_element).astype(int),opening_element).astype(int)

  #Applying the obtained mask into the intensities and selecting the range with maximum intensity.
  id_range_max_intensity = np.argmax(filtered_vector * intensities_vector_input)
  filtered_range = input_ranges_vector[id_range_max_intensity]

  return filtered_range

#Funtion name: closest_intensities_filter
#Inputs:
## intensities_vector_input -> vector that contains the integer intensities associated to each range.
## input_ranges_vector -> vector that contains the float ranges that have an intensity related.
## k_threshold-> input threshold parameter that sets the integer value to binarize the intensities.
## k_opening_size -> input parameter to select how many pixels are in the structure element used on the opening
## k_closing_size -> input parameter to select how many pixels are in the structure element used on the closing
#Output: 
##filtered_range -> the function outputs a float range value obtained after filtering
def closest_intensities_filter(intensities_vector_input,ranges ,k_threshold, k_opening_size, k_closing_size):
    #Applying a binary threhsold according to k_threshold parameter.
    binary_vector_input = (intensities_vector_input > k_threshold).astype(int)

    #Creating morphological filter structures
    opening_element = np.ones(k_opening_size)
    closing_element = np.ones(k_closing_size)

    #Applying first a closing with the specified structure element to connect detections.
    #After that, it is applied a opening with the specified structure element to eliminate detections with not enought size.
    filtered_vector = ndimage.binary_opening(ndimage.binary_closing(binary_vector_input).astype(int)).astype(int)
    
    #Applying the obtained mask into the intensities and selecting the closest filtered range.
    id_range = np.argmax(filtered_vector)
    filtered_range = ranges[id_range]
    return filtered_range

class IntensitiesFilterNode(object):
    def __init__(self):
	#Constructor for the Node object.
        self.namespace = rospy.get_namespace()[1:-1]
        rospy.loginfo('Intensities Filter Started')
	
	#Defining subscribers and publisher types
        self.intensities_input = LaserScan()
        self.filtered_range_msg = Range()

        self.time = rospy.get_time()

	#Setting the publisher topic
        self.pub_range = rospy.Publisher("/sparus2/imagenex_852_echosounder/echo_0_range", Range, queue_size = 1)

	#Setting the subscriber topic
        self.sub_vehicle_position = rospy.Subscriber('/sparus2/imagenex_852_echosounder/echo_0_intensities', LaserScan,self.cbk_intensities, queue_size = 1)

        #Filter Parameters
	self.k_opening_size = 1
	self.k_closing_size = 1
	self.k_threshold = 120

	#Comment/uncomment the mode of the filter
	self.mode = "max_filter"
	#self.mode = "closest_filter"

	rospy.loginfo('node initialized')

    def cbk_intensities(self, msg):
	#Callback function to procces the incoming subscriber messages
	
	#Assign msg to filter input parameters        
	self.intensities_input.ranges = msg.ranges
        self.intensities_input.intensities = msg.intensities
        self.time = rospy.get_time()
	
	#Call the filter depending on the selected mode	
	if self.mode == "max_filter":
		self.filtered_range_msg.range = max_intensities_filter(np.array(self.intensities_input.intensities), self.intensities_input.ranges ,self.k_threshold,self.k_opening_size, self.k_closing_size)
	
	if self.mode =="closest_filter":
		self.filtered_range_msg.range = closest_intensities_filter(np.array(self.intensities_input.intensities), self.intensities_input.ranges ,self.k_threshold,self.k_opening_size, self.k_closing_size)

	self.filtered_range_msg.header = msg.header

	#Publish the obtained results from filter
        self.pub_range.publish(self.filtered_range_msg)



if __name__ == '__main__':
    # initialize ROS node
    rospy.init_node('subdron_intensities_filter')
    # start node
    node = IntensitiesFilterNode()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10) #10Hz
    while not rospy.is_shutdown():

        rate.sleep()
