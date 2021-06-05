#!/usr/bin/env python2
"""
Simple node to comunicate with the echosounder.
"""

# ROS
import rospy

# Import messages from ROS
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# Import ping protocol library
from brping import Ping1D

# Numpy to help us create images
import numpy as np  

# To help us deal with images :)
from cv_bridge import CvBridge


class DriverEchosounder:

    def __init__(self):

        self.bridge = CvBridge()

        # Import parameters from launch file
        self.rate = rospy.Rate(rospy.get_param('~rate', 10))
        self.publish_image = rospy.get_param('~publish_image', True)
        baudrate = rospy.get_param('~baud', 115200)
        serial_port = rospy.get_param('~port', "/dev/echosounder")
        image_width = rospy.get_param('~image_width', 255)

        # Initialize the connection with the device
        self.echosounder = Ping1D(serial_port, baudrate)

        # Create a empty new image - height, width, 3(r, g, b)
        self.image = np.zeros([200, image_width, 3], dtype=np.uint8)

        # Define the publishers that we need 
        self.image_publisher = rospy.Publisher(
            'echosounder/image', Image, latch=True, queue_size=1)
 
        self.confidence_publisher = rospy.Publisher(
            'echosounder/confidence', Float32, latch=True, queue_size=1)

        self.distance_publisher = rospy.Publisher(
            'echosounder/distance', Float32, latch=True, queue_size=1)

        # Give it a bit of time for the pubs and subscribers to settle
        rospy.sleep(0.2)

    # ------------------------------------------------------------------
    def check_connection(self):
        """
        Try to stablish connection with the echosounder and checks 
        if the echosounder was successfully initialized.
        """         

        if self.echosounder.initialize() is True:
            rospy.loginfo("Echosounder successfully initialized.")
            return True
        else:
            rospy.logerr("Failed to initialize echosounder!")
            return False

    def acquire_data(self):
        """
        Requests the data from the echosounder. The echosounder returns a 
        dictionary with a lootttt of values..
        We only care about the distance, confidene and the intensities, and
        that is what we return.
        """

        # Requests the data from the echosounder
        data = self.echosounder.get_profile()

        # Converts the bytes and store everything in a list
        intensities = [k for k in bytearray(data["profile_data"])]

        return data["distance"], data["confidence"], intensities
        
    def create_image(self, intensities):
        """
        Creates a image based on the intensities provided by the echosounder.
        The origin of our image is the top left corner.  
        Origin of the image is (0, 0)    
        """

        # Shift the rows of the image right 
        self.image = np.roll(self.image, 1, axis=1)
        
        # Normalize intensities
        norm = [round(float(i)/max(intensities), 2) for i in intensities]
        
        # Anddddd give it a colour per point
        for i in range(0, len(norm)):

            point_color = norm[i]
 
            # height, width r, g, b
            self.image[i][0][0] = point_color * 255
            self.image[i][0][1] = (1-point_color) * 255 
            self.image[i][0][2] = (1-point_color) * 255 

    def publish_data(self, distance, confidence):
        """
        Simple function to help us managing the publishing of the data
        """

        # Convert distance from milimeters to meters
        distance_si = distance / 1000.0

        # Create and fill the messages
        distance_msg = Float32()
        distance_msg.data = distance_si
        confidence_msg = Float32()
        confidence_msg.data = confidence

        # And finally publish the data to the ros system
        self.distance_publisher.publish(distance_msg)
        self.confidence_publisher.publish(confidence_msg)

        if self.publish_image:

            # Create and fill the Image message
            img_msg = Image()
            img_msg = self.bridge.cv2_to_imgmsg(self.image, "rgb8")

            # And finally publish the data to the ros system
            self.image_publisher.publish(img_msg)


    # Entry point and main loop
    # ------------------------------------------------------------------
    def main(self):
        """
        This function calls all other functions that are necessary for 
        the smooth and effective operation of this ROS node.
        """

        rospy.loginfo('The echosounder driver node is now active.')

        # Checks if the connection with the echosounder was stablished
        connection = self.check_connection()

        # TODO: write a function to configure the echosounder
        # self.set_configuration()

        # Request and publish data in infinite loopc
        while connection and not rospy.is_shutdown():

            # Communicates with the echosounder - requests and receives data
            distance, confidence, intensities = self.acquire_data()

            # Creates an image based on the array of intensities received
            self.create_image(intensities)

            self.publish_data(distance, confidence)

            # Take a nap
            self.rate.sleep()

        rospy.loginfo('The echosounder driver node is no longer active.')


if __name__ == '__main__':

    # Initialize ROS node
    rospy.init_node('driver_echosounder')

    DE = DriverEchosounder()
    DE.main()

    rospy.spin()
