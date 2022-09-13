import collections
import operator
import os
import time

import numpy as np
import rclpy
import tensorflow as tf
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

class FPFH_Classifier(Node):
    """
    Create a classifier node
    """

    def __init__(self, offset, model, sub_num):

        super().__init__(f'classifier_node_{sub_num}')

        #self.declare_parameter('cycles', 14)

        self.model = model
        self.sub_num = sub_num
        self.offset = offset
        self.memory = collections.deque([])
        #self.cycles = self.get_parameter('cycles').value
        self.counter = 0




        self.subscription = self.create_subscription(
                    Float64MultiArray,
                    f"fpfh_relay_node_{self.sub_num-1}",
                    self.histogram_callback,
                    1)
 

        self.publisher = self.create_publisher(
            String, f"classifier_node_{self.sub_num}", 1)

        # variable to set whether or not histograms are forwarded.
        # It is set to false after a time shift has been introduced 
        self.shift_classifier = True

        self.relay = self.create_publisher(
            Float64MultiArray, f"fpfh_relay_node_{self.sub_num}", 1)

    def histogram_callback(self, histogram):
        # use a counter to keep track how many times the subscriber has received a message

        self.counter = self.counter + 1

        # remove time shifting after 1 frame has been received
        # the shift introduced this way is equal to the processing time
        # of the first frame by the fpfh publisher node 
        if self.counter >= 2:
            self.shift_classifier = False

        if not self.shift_classifier: 
            self.relay.publish(histogram)

        self.memory.append(histogram.data)
        
        if(self.counter == self.offset):
            self.send_to_lstm(self.memory)
            self.memory.clear()

    def send_to_lstm(self, data):
        npdata = np.array(data)[np.newaxis, :, :]
        output = self.evaluate_prediction(npdata)

        self.publisher.publish(output)

    def evaluate_prediction(self, data):
        prediction_output = self.run_single_prediction(data)
        prediction_dict = prediction_output['prediction']
        # get the most confident Prediction from the dict
        result = max(prediction_dict.items(), key=operator.itemgetter(1))

        output = String()

        output.data = str(self.sub_num) + ":" + str(result[0]) + ":" + str(result[1])
        return output

    def run_single_prediction(self, data):

        
        self.model.reset_states()
        self.counter = 0

        prediction = self.model.predict(data).tolist()
        result = {
            'prediction': {
                'Thumbs Up': prediction[0][0],
                'Thumbs Down': prediction[0][1],
                'Swipe Left': prediction[0][2],
                'Swipe Right': prediction[0][3],
                'One Snap': prediction[0][4],
                'Two Snaps': prediction[0][5]
            }
        }

        return result


def main(args=None):

    dirname = os.path.dirname(__file__)
    path = os.path.join(dirname, 'savedModel/fpfh_model.h5')

    model = tf.keras.models.load_model(path)

    offset = 14
    sub_num = 12
    

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create a subscriber
    fpfh_classifier = FPFH_Classifier(offset, model, sub_num)
    
    # # Spin in a separate thread
    rclpy.spin(fpfh_classifier)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fpfh_classifier.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
