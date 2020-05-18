#!/usr/bin/env python

from os import path, makedirs, rename

import rospy
import airsim
import numpy as np
import datetime

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
import cv2


class ImageS_Helper:

    def __init__(self, save_folder):
        self.subscriber = rospy.Subscriber("/camera/images", CompressedImage, self.on_image_published, queue_size=3)
        self.stop_subscriber = rospy.Subscriber('/stop_message', Bool, self.stop_publish, queue_size=1)

        self.save_imgs_folder = save_folder
        self.image_type_dict = {
            airsim.ImageType.Scene: 'scene', 
            airsim.ImageType.Segmentation: 'segmentation',
            airsim.ImageType.DepthVis: 'depth'
        }
    
    def stop_publish(self, msg):
        print(msg)
        rospy.signal_shutdown('received stop message')

    def on_image_published(self, msg):
        stamp = msg.header.stamp
        # seq = msg.header.seq
        head_info = msg.header.frame_id.split()
        seq, fid = int(head_info[0]), head_info[1]

        form = msg.format
        data = msg.data
        print(seq, fid)

        save_folder = path.join(self.save_imgs_folder, self.image_type_dict[seq])
        print(save_folder)
        if not path.exists(save_folder):
            makedirs(save_folder)
        filename = path.join(save_folder, "img_{0}_{1}.jpg".format(seq, fid))

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        p = cv2.imwrite(filename, image_np)
        print(p)


if __name__ == "__main__":
    ih = ImageS_Helper("/home/mpcutino/test")

    rospy.init_node('image_subscriber', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image saving module")
    cv2.destroyAllWindows()
