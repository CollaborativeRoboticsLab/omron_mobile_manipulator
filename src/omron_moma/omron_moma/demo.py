import rclpy
import time
import sys
import json
from ament_index_python.packages import get_package_share_directory
from rclpy import node
moma_share = get_package_share_directory('omron_moma')
pp_library =  get_package_share_directory('pickplace') + '/pickplace/pp_library'

from pp_library import IO, Transform, Script, Move, Modbus
from om_aiv_navigation.goto_goal import AmrActionClient
from pickplace_msgs.srv import AskModbus
from rcl_interfaces.srv import SetParameters

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
    def __init__(self, node):
        self.node = node
        self.tf = Transform.TransformClass()
        self.io = IO.IOClass()
        self.mover = Move.MoveClass()
        self.listener = Script.ScriptClass()

        self.cli = node.create_client(AskModbus, 'ask_modbus')

        self.listener.wait_tm_connect()


    def execute_tm(self, coord):
        self.tf.add_vbases(coord.vbase_pick, coord.vbase_place)

        self.mover.set_position(coord.view_pick)
        pick, safepick = get_positions(self.listener, self.node, self.cli, self.tf, "vbase_pick", coord.vjob_name)
        self.mover.set_position(safepick)
        self.io.open()
        self.mover.set_position(pick)
        self.io.close()
        self.mover.set_position(safepick)

        self.mover.set_position(coord.view_place)
        place, safeplace = get_positions(self.listener, self.node, self.cli, self.tf, "vbase_place", coord.vjob_name)
        self.mover.set_position(safeplace)
        self.mover.set_position(place)
        self.io.open()
        self.mover.set_position(safeplace)

        self.mover.set_position(coord.home_pos)

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

    tm_handler = TMHandler(node)
    action_client = AmrActionClient()

    # Load the coordinates taught in teach_setup for the respective goals
    Goal1_coords = Coordinates('Goal1')
    Goal2_coords = Coordinates('Goal2')

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

    except KeyboardInterrupt:
            node.get_logger().info("Program shut down!")

    
    

if __name__ == '__main__':
    main()

