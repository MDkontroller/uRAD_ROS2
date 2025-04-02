import rclpy
import urad.uRAD_RP_SDK11 as uRAD_RP_SDK11
from rclpy.node import Node
from interfaces.msg import CwIq
import yaml
import os


class urad_config():

    def __init__(self):
        super().__init__('urad_publisher')

        # Load parameters from YAML
        self.load_params_from_yaml()
        # Initialize radar
        self.initialize_radar()
        # Get the path to the config file
        # config_path = os.path.join(os.path.dirn
        #                            me(os.path.dirname(__file__)), 'config', 'radar_parameters.yaml')
        
        # # Read parameters from YAML file
        # with open(config_path, 'r') as file:
        #     config_data = yaml.safe_load(file)
        #     self.parameters = config_data['radar_parameters']

        # self.turn_on  = uRAD_RP_SDK11.turnON()
        # #self.config   = uRAD_RP_SDK11.loadConfiguration(mode, f0, BW, Ns, Ntar, Rmax, MTI, Mth, Alpha, distance_true, velocity_true, SNR_true, I_true, Q_true, movement_true)
        # self.config   = uRAD_RP_SDK11.loadConfiguration(**self.parameters)
        # if (self.config != 0):
        #     #print('config unsuccesfull :( ')
        #     self.get_logger().info('uRAD_RP_SDK11.loadConfiguration: config unsuccesfull :(   ')
        #     self.closeProgram()

    def load_params_from_yaml(self):
        # Path to your YAML file
        yaml_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'urad_config.yaml')
        
        try:

            with open(yaml_path, 'r') as file:
                self.params = yaml.safe_load(file)
                self.get_logger().info(f'Loaded parameters from {yaml_path}')

        except Exception as e:

            self.get_logger().error(f'Failed to load parameters from YAML: {e}')
            self.params = {
                'f0': 125, 'BW': 0, 'Ns': 200, 'I_true': True, 'Q_true': True,
                'Ntar': 4, 'MTI': 0, 'Mth': 0, 'Rmax': 100, 'Alpha': 5,
                'distance_true': False, 'velocity_true': True, 'SNR_true': True,
                'movement_true': False, 'mode': 1
            }
            self.get_logger().info('Using default parameters')

    def initialize_radar(self):
        # Turn on uRAD
        self.turn_on = uRAD_RP_SDK11.turnON()
        
        # Load configuration
        self.config = uRAD_RP_SDK11.loadConfiguration(**self.params)
        if (self.config != 0):
            self.get_logger().error('uRAD_RP_SDK11.loadConfiguration: config unsuccessful')
            self.close_program()




    def close_program(self):
        # First set the shutdown flag to True to ensure the main loop will exit
        self.should_shutdown = True
        # Switch OFF uRAD
        try:
            uRAD_RP_SDK11.turnOFF()
        except Exception as e:
            self.get_logger().error(f'Error turning off radar: {e}')
        finally:
            # node will be shutdown in the main function
            pass


class Publisher(Node):

    def __init__(self):
        super().__init__('pub_node')
        self.publisher = self.create_publisher(CwIq, 'cw_iq', 3)
        self.i     = 0

    def close_program(self):
        # switch OFF uRAD
        return_code = uRAD_RP_SDK11.turnOFF()
        exit()

    def publish_radar(self):

        msg      = CwIq()
        return_code, results, raw_results   = uRAD_RP_SDK11.detection()    
        msg.i    = raw_results[0]
        msg.q    = raw_results[1]

        if (return_code != 0):
            print('something went wrong with this sensor!')
            self.close_program()

        self.publisher.publish(msg)

        self.get_logger().info('publishing.. %d' % self.i )
        self.get_logger().info('%s' % msg.q)
        self.i   += 1

def main(args=None):

    rclpy.init(args=args)
    urad_config()
    publisher_s = Publisher()
    #rclpy.spin(publisher_s) # spin for timer

    while rclpy.ok():
        publisher_s.publish_radar()

    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

