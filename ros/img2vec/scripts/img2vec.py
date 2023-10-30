#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError
import cv2
 
import torch
import torch.nn as nn
import torchvision.models as models
import torchvision.transforms as transforms
from torch.autograd import Variable
from PIL import Image
 
class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_embeddings",numpy_msg, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_raw",Image,self.callback)

    self.model = models.resnet18(pretrained=True) 
    self.layer = self.model._modules.get('avgpool')
    self.model.eval()

    self.scaler = transforms.Scale((224, 224))
    self.normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                    std=[0.229, 0.224, 0.225])
    self.to_tensor = transforms.ToTensor()


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    image_embeddings = get_vector(cv_image)
    print("embedding generetaed")
    print(image_embeddings.size())
    img_embedding_np = image_embeddings.numpy()

    print("published, printing first ten:  ")
    print(img_embedding_np[:10])
    try:
      self.image_pub.publish(img_embedding_np)
    except CvBridgeError as e:
      print(e)
# source: https://becominghuman.ai/extract-a-feature-vector-for-any-image-with-pytorch-9717561d1d4c
    def get_vector(image):
        # 1. Load the image with Pillow library
        t_img = Variable(self.normalize(self.to_tensor(self.scaler(image))).unsqueeze(0))    
        my_embedding = torch.zeros(512)    # 4. Define a function that will copy the output of a layer
        def copy_data(m, i, o):
            my_embedding.copy_(o.data)    # 5. Attach that function to our selected layer
        h = self.layer.register_forward_hook(copy_data)    # 6. Run the model on our transformed image
        self.model(t_img)    # 7. Detach our copy function from the layer
        h.remove()    # 8. Return the feature vector
        return my_embedding

def main():
  ic = image_converter()
  print("initializing img2vec")
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
