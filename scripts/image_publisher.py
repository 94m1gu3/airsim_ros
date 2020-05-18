#!/usr/bin/env python

import rospy
import airsim
import numpy as np

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
import cv2


class ImageP_Helper:

    def __init__(self):
        self.def_publisher = rospy.Publisher("/camera/images", CompressedImage, queue_size=3)
        self.stop_subscriber = rospy.Subscriber('/stop_message', Bool, self.stop_publish, queue_size=1)

    def publish(self, responses, frame_id):
        for response in responses:
            img = get_image(response)
            # print(response.image_type)
            msg = self.getCompressedImg(img, response.compress, response.image_type, frame_id)
            self.def_publisher.publish(msg)
    
    def stop_publish(self, msg):
        print(msg)
        print('signal received. Shutting down')
        rospy.signal_shutdown('received stop message')
    
    def getCompressedImg(self, image_data, is_compressed, seq, frame_id):
        if not is_compressed:
            image_data = np.array(cv2.imencode('.jpg', image_data)[1]).tostring()
        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = seq
        msg.header.frame_id = " ".join([str(seq), str(frame_id)])

        msg.format = "jpg"
        msg.data = image_data
        return msg


def get_image(response):
    if response.pixels_as_float:
        return airsim.get_pfm_array(response)
    elif response.compress: #png format
        return response.image_data_uint8
    else: #uncompressed array
        # print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
        img_rgb = img1d.reshape(response.height, response.width, 4) # reshape array to 4 channel image array H X W X 3
        return img_rgb


if __name__ == "__main__":
    client = airsim.MultirotorClient()
    client.confirmConnection()
    # client.enableApiControl(True)
    # client.armDisarm(True)
    rospy.init_node('image_publisher', anonymous=True, disable_signals=True)
    rate = rospy.Rate(10) # 30hz

    def myhook():
        print("shutdown time!")

    rospy.on_shutdown(myhook)
    
    helper = ImageP_Helper()
    count = 0
    while not rospy.is_shutdown():
        responses = client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
            airsim.ImageRequest("0", airsim.ImageType.Segmentation, True), # segmentation
            airsim.ImageRequest("0", airsim.ImageType.Scene), #scene vision image
            ])

        helper.publish(responses, count)
        count += 1

        rate.sleep()
    print("shutdown work")
    # client.armDisarm(False)
    # client.reset()

    # # that's enough fun for now. let's quit cleanly
    # client.enableApiControl(False)

