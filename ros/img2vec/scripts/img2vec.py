#!/usr/bin/env python
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