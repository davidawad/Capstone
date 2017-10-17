from styx_msgs.msg import TrafficLight
from keras import models
import numpy as np
import tensorflow as tf

class TLClassifier(object):
    RED_CLASS = np.array([1.,0.,0.,0.], dtype=np.float32)
    YELLOW_CLASS = np.array([0.,1.,0.,0.], dtype=np.float32)
    GREEN_CLASS = np.array([0.,0.,1.,0.], dtype=np.float32)
    

    def __init__(self):
        self.model = models.load_model('./tl_model.h5')
        self.graph = tf.get_default_graph()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        xs = image[None,:]
        with self.graph.as_default():
            ys = self.model.predict(xs)

        result = ys[0]
        if np.allclose(result, TLClassifier.RED_CLASS, atol=1.e-4):
            return TrafficLight.RED 
        elif np.allclose(result, TLClassifier.YELLOW_CLASS, atol=1.e-4):
            return TrafficLight.YELLOW
        elif np.allclose(result, TLClassifier.GREEN_CLASS, atol=1.e-4):
            return TrafficLight.GREEN
        else:
            return TrafficLight.UNKNOWN
