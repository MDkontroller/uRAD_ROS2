import rclpy
import urad.uRAD_RP_SDK11 as uRAD_RP_SDK11
from rclpy.node import Node
from interfaces.msg import CwIq


class urad_config():

    def __init__(self):
        
        self.parameters = {"f0":125, "BW":0, "Ns":200, "I_true":"True",
         "Q_true":True, "Ntar":4, "MTI":0, "Mth":0,"Rmax":100,
         "Alpha":5, "distance_true":False, "velocity_true":True,
         "SNR_true":True,"movement_true":False, "mode":1}

        self.turn_on  = uRAD_RP_SDK11.turnON()
        #self.config   = uRAD_RP_SDK11.loadConfiguration(mode, f0, BW, Ns, Ntar, Rmax, MTI, Mth, Alpha, distance_true, velocity_true, SNR_true, I_true, Q_true, movement_true)
        self.config   = uRAD_RP_SDK11.loadConfiguration(**self.parameters)
        if (self.config != 0):
            self.get_logger().info('uRAD_RP_SDK11.loadConfiguration: config unsuccesfull :(   ')
            exit()
    

class Publisher(Node):

    def __init__(self):
        super().__init__('pub_node')
        self.publisher = self.create_publisher(CwIq, 'cw_iq', 3)
        repetition = 0.5
        self.timer = self.create_timer(repetition, self.timer_callback)
        self.i     = 0

    def closeProgram(self):
        # switch OFF uRAD
        return_code = uRAD_RP_SDK11.turnOFF()
        exit()

    def timer_callback(self):

        msg      = CwIq()
        return_code, results, raw_results   = uRAD_RP_SDK11.detection()    
        msg.i    = raw_results[0]
        msg.q    = raw_results[1]

        if (return_code != 0):
            print('something went wrong with this sensor!')
            self.closeProgram()

        self.publisher.publish(msg)

        self.get_logger().info('publishing.. %d' % self.i )
        self.get_logger().info('%s' % msg.q)
        self.i   += 1

def main(args=None):

    rclpy.init(args=args)
    urad_config()
    publisher_s = Publisher()
    rclpy.spin(publisher_s)

if __name__ == '__main__':
    main()

