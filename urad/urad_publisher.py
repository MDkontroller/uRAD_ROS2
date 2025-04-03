import rclpy
import urad.uRAD_RP_SDK11 as uRAD_RP_SDK11
from rclpy.node import Node
from interfaces.msg import CwIq
from urad_base import UradBaseNode

class UradPublisher(UradBaseNode):


    def __init__(self):
        super().__init__('urad_publisher')

        # Create publisher
        self.publisher = self.create_publisher(CwIq, 'cw_iq', 10)
       
        # Create timer for publishing
        self.timer = self.create_timer(0.1, self.publish_radar)  # 10Hz
        self.i = 0
    
       
    def publish_radar(self):
            # Check if we should stop
            if self.should_shutdown:
                return
                
            try:
                msg = CwIq()
                return_code, results, raw_results = uRAD_RP_SDK11.detection()
            
                if (return_code != 0):
                    self.get_logger().error(f'Detection error: Data delivery failed! Code: {return_code}')
                    self.close_program()
                    return
                
                msg.i = raw_results[0]
                msg.q = raw_results[1]
            
                self.publisher.publish(msg)
                self.get_logger().info(f'Publishing.. {self.i}')
                self.i += 1
            except Exception as e:
                self.get_logger().error(f'Error during radar detection: {e}')
                self.close_program()
       

def main(args=None):

    rclpy.init(args=args)
    urad_publisher = UradPublisher()
   
    try:
        while rclpy.ok() and not urad_publisher.should_shutdown:
            rclpy.spin_once(urad_publisher)
    except Exception as e:
        urad_publisher.get_logger().error(f'Error in main loop: {e}')
    
    finally:

        if not urad_publisher.should_shutdown:
            urad_publisher.close_program()    # prevent to shutdown sensor twice

        urad_publisher.destroy_node()  # only destroys node
        #rclpy.shutdown()              to shutdown  ROS2 system


if __name__ == '__main__':
    main()