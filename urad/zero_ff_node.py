import rclpy
from rclpy.node import Node
from interfaces.msg import CWIQZERO, CwIq
import numpy as np

#from urad.msg import iq_cw

class Zero_ff_Extractor(Node):

    def __init__(self):
        super().__init__('zero_ff_node')

        self.subscribtion = self.create_subscription(CwIq, 'cw_iq', self.callback, 5 )
        self.publisher    = self.create_publisher(CWIQZERO, 'cw_iq_offsett', 3)
        self.get_logger().info('node ready ...')
        self.MAX_VOLT     =   3.3
        self.ADC_INTERVALS = 4096
        self.counter = 0

    def remove_zero_frequencies(self, signal):

        return np.subtract(np.multiply(signal, self.MAX_VOLT/self.ADC_INTERVALS), np.mean(np.multiply(signal, self.MAX_VOLT/self.ADC_INTERVALS)))

    def callback(self,msg):

        i = np.asarray(msg.i)
        q = np.asarray(msg.q)

        i_p = (self.remove_zero_frequencies(i)).tolist()
        q_p = (self.remove_zero_frequencies(q)).tolist()

        self.get_logger().info(f' publishing len q_p: {len(q_p)} counter= {self.counter}')

        msg_zero = CWIQZERO()

        msg_zero.i_p = i_p
        msg_zero.q_p = q_p

        self.publisher.publish(msg_zero)

        #Cplex = i + 1j*q  # need to create a .msg for complex data
        self.counter += 1



def main(args=None):

    rclpy.init(args=args)
    node = Zero_ff_Extractor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()