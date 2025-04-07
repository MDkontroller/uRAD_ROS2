import rclpy
import urad.uRAD_RP_SDK11 as uRAD_RP_SDK11
from interfaces.msg import CwIq
from urad.urad_base import UradBaseNode


class UradPubStreamer(UradBaseNode):

    def __init__(self):

        super().__init__('urad_streamer')

        self.publisher = self.create_publisher(CwIq, 'cw_iq', 3)
        self.should_shutdown = False
        self.i     = 0

    def publish_radar(self):

        try:
            msg      = CwIq()
            return_code, results, raw_results   = uRAD_RP_SDK11.detection()    
            msg.i    = raw_results[0]
            msg.q    = raw_results[1]

            if (return_code != 0):
                self.get_logger().error(f'Datastream failed!')
                self.close_program()
                return

            self.publisher.publish(msg)
            self.i   += 1
            
            # Log only occasionally to avoid excessive logs
            if self.i % 100 == 0:
                self.get_logger().info(f'Publishing.. {self.i}')
                self.get_logger().info('%s' % msg.q)

        except Exception as e:
                self.get_logger().error(f'Error during data delivery!! {e}')
                self.close_program()



def main(args=None):

    rclpy.init(args=args)
    publisher_node = UradPubStreamer()
   
    try:
        while rclpy.ok() and not publisher_node.should_shutdown:
            publisher_node.publish_radar() # just publish, no callbacks

    except Exception as e:
        publisher_node.get_logger().error(f'Error in main loop: {e}')
    
    finally:

        if not publisher_node.should_shutdown:
            publisher_node.close_program()    # prevent to shutdown sensor twice

        publisher_node.destroy_node()  # only destroys node
        #rclpy.shutdown()              to shutdown  ROS2 system


if __name__ == '__main__':
    main()

