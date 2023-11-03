#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from img2vec.msg import ImgEmbedding

import torch
import torch.nn as nn
import torchvision.models as models
import torchvision.transforms as transforms
from torch.autograd import Variable

 
class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_embeddings",ImgEmbedding, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_raw",Image,self.callback)

    self.model = models.resnet18(pretrained=True) 
    self.layer = self.model._modules.get('avgpool')
    self.model.eval()

    self.scaler = transforms.Resize((224, 224))
    self.normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                    std=[0.229, 0.224, 0.225])
    self.to_tensor = transforms.ToTensor()


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    image_embeddings = get_vector(cv_image)
    rospy.logdebug("Image embedggins generated with size: %s", image_embeddings.size())
    
    # convert back to array of 64 floats
    img_embedding_np = image_embeddings.numpy()
    img_embedding_np = np.array(img_embedding_np, dtype='float64')
    rospy.logdebug(img_embedding_np[:10])
    rospy.logdebug("type: %s", type(img_embedding_np))
    try:
      self.image_pub.publish(img_embedding_np)
    except CvBridgeError as e:
      print(e)
# source: https://becominghuman.ai/extract-a-feature-vector-for-any-image-with-pytorch-9717561d1d4c
    def get_vector(image):
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
