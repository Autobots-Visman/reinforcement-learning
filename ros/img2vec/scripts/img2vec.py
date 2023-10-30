#!/usr/bin/env python
<<<<<<< HEAD
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
 
 
class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera_topic",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)



def main():
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
=======
import rospy
from std_msgs.msg import String



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    pub.publish("test succeeded with: " + data.data)
def listener():
    rospy.init_node('img2vec', anonymous=True)
    rospy.Subscriber("cemra_topic", String, callback)
    global pub
    pub = rospy.Publisher('img_embeddings', String, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
>>>>>>> 141ef0f806a77ac899520bd920e20c4505ead37c
