import rclpy
from rclpy.node import Node
from interfaces.msg import CWIQZERO, CwIq
import urad.uRAD_RP_SDK11 as uRAD_RP_SDK11
from urad_base import UradBaseNode
import numpy as np
import os
import yaml
from ament_index_python.packages import get_package_share_directory

class Zero_ff_Extractor(UradBaseNode):

    def __init__(self):
        super().__init__('zero_ff_node')

        self.subscribtion = self.create_subscription(CwIq, 'cw_iq', self.callback, 5 )
        self.publisher    = self.create_publisher(CWIQZERO, 'cw_iq_offsett', 3)
        self.MAX_VOLT      =   3.3
        self.ADC_INTERVALS = 4096
        self.counter       = 0
        self.get_logger().info('Zero frequency extractor node is ready')


    def remove_zero_frequencies(self, signal):

        try:
            # Convert to voltage scale
            voltage_signal = np.multiply(signal, self.MAX_VOLT/self.ADC_INTERVALS)
            # Remove DC offset (mean value)
            return np.subtract(voltage_signal, np.mean(voltage_signal))
        except Exception as e:
            self.get_logger().error(f'Error in signal processing: {e}')
            return np.zeros_like(signal)  # Return zeros in case of error

         #return np.subtract(np.multiply(signal, self.MAX_VOLT/self.ADC_INTERVALS), np.mean(np.multiply(signal, self.MAX_VOLT/self.ADC_INTERVALS)))

    def callback(self,msg):

        try: 

            i = np.asarray(msg.i)
            q = np.asarray(msg.q)

            i_p = (self.remove_zero_frequencies(i)).tolist()
            q_p = (self.remove_zero_frequencies(q)).tolist()

            # building message 
            msg_zero = CWIQZERO()

            msg_zero.i_p = i_p
            msg_zero.q_p = q_p

            self.publisher.publish(msg_zero)

            #Cplex = i + 1j*q  # need to create a .msg for complex data '(TODO )

                        # Log info periodically (every 50 messages)
            if self.counter % 50 == 0:
                    self.get_logger().info(f'Publishing processed I/Q data: {len(q_p)} samples, counter={self.counter}')
                
            self.counter += 1

        except Exception as e:
            self.get_logger().error(f'Error processing I/Q data: {e}')




def main(args=None):

    rclpy.init(args=args)
    node = Zero_ff_Extractor()
   
    try:
        while rclpy.ok() and not node.should_shutdown:
         rclpy.spin_once(node)
    except Exception as e:
        node.get_logger().error(f"Error in main {e}")
    finally:

        if not node.should_shutdown:
            node.close_program()  # prevent to shutdown sensor twice

        node.destroy_node()  # only destroys node
        #rclpy.shutdown()    # to shutdown  ROS2 system

if __name__ == '__main__':
    main()