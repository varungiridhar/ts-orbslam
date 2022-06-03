#!/usr/bin/env python2.7
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
import sys
import numpy as np


VERBOSE=False 

class FlipCam:
   

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        pub_name = rospy.get_param('~cam_output', '/terrasentia/front_cam_node/rotated/compressed')
        self.flip_image = rospy.Publisher(pub_name, CompressedImage, queue_size = 1)
        
        # subscribed Topic
        sub_name = rospy.get_param('~cam_input', '/terrasentia/front_cam_node/image_raw/compressed')
        self.subscriber = rospy.Subscriber(sub_name, CompressedImage, self.callback,  queue_size = 1, buff_size = 2**24)

        self.disable_vision = rospy.get_param('disable_vision', False)
        self.error_print_time = rospy.Time.now().to_sec()

        rospy.loginfo(str(rospy.get_name()) + " subscribed to " + str(sub_name) + " and publishing to " + str(pub_name) + ". disable_vision? " + str(self.disable_vision))   

    def callback(self, ros_data):
        start_time = rospy.get_time()
        if self.disable_vision:
            if rospy.Time.now().to_sec() - self.error_print_time > 30:
                self.error_print_time = rospy.Time.now().to_sec() 
                rospy.logerr('front camera is not being published. Change disable_vision in terrasentia_ag_nav/launch/run_navigation.launch')
            return -1
        if VERBOSE :
            print('received image of type: "%s"' % ros_data.format)
       
         #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        #flip_image = cv2.flip(image,0)
        flip_image = cv2.rotate(image, cv2.ROTATE_180)
        msg = CompressedImage()
        msg.header.frame_id = ros_data.header.frame_id
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg',flip_image)[1]).tostring()
        #rospy.loginfo("--->Took " + str(rospy.get_time()-start_time) + ' to rotate image')
        #### Publish numpy topic  ####
        self.flip_image.publish(msg)
        #rospy.loginfo("Took " + str(rospy.get_time()-start_time) + ' until publish')
       

def main():
    rospy.init_node('flip_cam', anonymous=True)
    fc = FlipCam()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down Flip cam")
 

if __name__ == '__main__':
    sys.exit(main() or 0)

