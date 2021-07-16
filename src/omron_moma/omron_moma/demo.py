import rclpy
import time
import sys
import json
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from rclpy import node
from std_msgs.msg import Bool
from rclpy.duration import Duration
moma_share = get_package_share_directory('omron_moma')
pp_library =  get_package_share_directory('pickplace') + '/pickplace/pp_library'

from pp_library import Pickplace_Driver, Transform
from om_aiv_navigation.goto_goal import AmrActionClient
from pickplace_msgs.srv import AskModbus
from pickplace_msgs.msg import MoveCube
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# Get the coordinates of the new vision base
def get_base(node, cli):
    while not cli.wait_for_service(timeout_sec=1.0):
        print("Service not available...")
    req = AskModbus.Request()
    req.req = 'get_base'
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result().position


# Get the new pick or place positions w.r.t the new vision base
def get_positions(listener, node, cli, tf, vbase_name, vjob_name):
    listener.exit_script()
    listener.change_base(vjob_name)
    time.sleep(0.1)
    new_vbase = get_base(node, cli)
    time.sleep(0.1)
    listener.change_base("RobotBase")
    time.sleep(0.1)
    if (vbase_name == "vbase_pick"):
        return tf.get_picks(new_vbase, vbase_name)
    elif (vbase_name == "vbase_place"):
        return tf.get_places(new_vbase, vbase_name)
    else:
        return new_vbase

# Set the destination transform location
def call_set_parameters(node, coordinates):
    # create client
    client = node.create_client(
        SetParameters,
        'destination_node/set_parameters'.format_map(locals()))

    # call as soon as ready
    ready = client.wait_for_service(timeout_sec=5.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')

    request = SetParameters.Request()
    param_values = ParameterValue(type = ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value = coordinates)
    request.parameters = [Parameter(name = 'destination_param', value = param_values)]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future) 

    # handle response
    response = future.result()
    if response is None:
        e = future.exception()
        raise RuntimeError(
            'Exception while calling service of node '
            "'{args.node_name}': {e}".format_map(locals()))
    return response

# Gets the transform between two frames in the transform tree
def get_lookup_transform(node, source, target):
    temp_buffer = tf2_ros.Buffer()
    dur = Duration()
    dur.sec = 10
    dur.nsec = 0
    temp_listener = tf2_ros.TransformListener(buffer=temp_buffer, node=node, spin_thread=True)
    time.sleep(1.0)
    temp_tf = Transform.TransformClass()
    while rclpy.ok():
        try:
            node.get_logger().info("unbork")
            temp_buffer.wait_for_transform_async(target, source, node.get_clock().now().to_msg())
            transform = temp_buffer.lookup_transform(target, source, node.get_clock().now().to_msg(), dur)
            print(transform)
            if transform is not None:
                node.get_logger().info("unborkened")
                break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            node.get_logger().info("bork")
        node.get_logger().info("While loop2")
        
    return temp_tf.stamped_to_euler(transform)
    
# Creates a class for coordinates from the teach_setup config.txt to be initialised
# The paramater 'mode' will be either 'load' or 'unload'
class Coordinates:
    def __init__(self, mode):
        with open(moma_share + '/' + mode + '_config.txt') as json_file:
            self.data = json.load(json_file)
            self.home_pos = self.data['home_pos']
            self.vjob_name = self.data['vjob_name']
            self.view_pick = self.data['view_pick']
            self.view_place = self.data['view_place']
            self.vbase_pick = self.data['vbase_pick']
            self.vbase_place = self.data['vbase_place']

class TMHandler:
    def __init__(self, node, pickplace_driver):
        self.node = node
        self.pickplace_driver = pickplace_driver
        self.tf = Transform.TransformClass()
        self.cli = node.create_client(AskModbus, 'ask_modbus')
        self.flagpublisher = self.node.create_publisher(MoveCube, 'objectflag', 10)

        self.pickplace_driver.wait_tm_connect()
        
    # Executes the pickplace sequence at the designated goal
    def execute_tm(self, coord):
        self.tf.add_vbases(coord.vbase_pick, coord.vbase_place)

        self.pickplace_driver.set_position(coord.home_pos)

        self.pickplace_driver.set_position(coord.view_pick)
        if not self.pickplace_driver.error:
            pick, safepick = get_positions(self.pickplace_driver, self.node, self.cli, self.tf, "vbase_pick", coord.vjob_name)
            call_set_parameters(self.node, pick)
            msg = MoveCube()
            msg.parent = "tm_base"
            msg.coordinates = pick
            self.flagpublisher.publish(msg)
            self.pickplace_driver.set_position(safepick)
            self.pickplace_driver.open()
            self.pickplace_driver.set_position(pick)
            self.pickplace_driver.close()
            msg.parent = "EOAT"
            msg.coordinates = pick
            self.flagpublisher.publish(msg)
            self.pickplace_driver.set_position(safepick)

        self.pickplace_driver.set_position(coord.view_place)
        if not self.pickplace_driver.error:
            place, safeplace = get_positions(self.pickplace_driver, self.node, self.cli, self.tf, "vbase_place", coord.vjob_name)
            call_set_parameters(self.node, place)
            self.pickplace_driver.set_position(safeplace)
            self.pickplace_driver.set_position(place)
            self.pickplace_driver.open()
            msg = MoveCube()
            msg.parent = "tm_base"
            msg.coordinates = place
            self.flagpublisher.publish(msg)
            self.pickplace_driver.set_position(safeplace)

        self.pickplace_driver.set_position(coord.home_pos)

        if self.pickplace_driver.error:
            self.node.get_logger().info("TM ERROR, SHUTTING DOWN PROGRAM!")
            exit()

def main():
    rclpy.init()
    node = rclpy.create_node('demo_node')

    # Initialise gripper using modbus
    cli = node.create_client(AskModbus, 'ask_modbus')
    while not cli.wait_for_service(timeout_sec=1.0):
        print("Service not available...")
    req = AskModbus.Request()
    req.req = 'init_io'
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    pickplace_driver = Pickplace_Driver.PickPlaceClass()
    tm_handler = TMHandler(node, pickplace_driver)
    action_client = AmrActionClient()

    # Load the coordinates taught in teach_setup for the respective goals
    Goal1_coords = Coordinates('Goal1')
    Goal2_coords = Coordinates('Goal2')

    # Set the TM to move to the designated home position
    pickplace_driver.set_position(Goal1_coords.home_pos)
    
    try:     
        goal2result = action_client.send_goal('Goal2')
        if not ("Arrived at" in goal2result):
            node.get_logger().info("Failed to arrive at goal!")
            exit()

        tm_handler.execute_tm(Goal2_coords)
        
        goal1result = action_client.send_goal('Goal1')
        if not ("Arrived at" in goal1result):
            node.get_logger().info("Failed to arrive at goal!")
            exit()

        tm_handler.execute_tm(Goal1_coords)
        zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        call_set_parameters(node, zero)
        flagpublisher = node.create_publisher(MoveCube, 'objectflag', 10)
        #temp_transform = get_lookup_transform(node, "world", "marker")
        msg = MoveCube()
        msg.parent = "world"
        msg.coordinates = zero
        flagpublisher.publish(msg)

    except KeyboardInterrupt:
            node.get_logger().info("Program shut down!")

    
    

if __name__ == '__main__':
    main()

