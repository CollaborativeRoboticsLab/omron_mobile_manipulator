import rclpy
import time
import sys
import json
from ament_index_python.packages import get_package_share_directory
from rclpy import node
moma_share = get_package_share_directory('omron_moma')
pp_library =  get_package_share_directory('pickplace') + '/pickplace/pp_library'

from pp_library import IO, Transform, Script, Move
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


def main():
    rclpy.init()
    pickplace_node = rclpy.create_node('pickplace_node')
    cli = pickplace_node.create_client(AskModbus, 'ask_modbus')
    vjob_name = ""
    view_pick = []
    view_place = []
    vbase_pick = []
    vbase_place = []
    with open(moma_share + '/config.txt') as json_file:
        data = json.load(json_file)
        home_pos = data['home_pos']
        vjob_name = data['vjob_name']
        view_pick = data['view_pick']
        view_place = data['view_place']
        vbase_pick = data['vbase_pick']
        vbase_place = data['vbase_place']

    pickplace_node.get_logger().info("Successfully obtained coordinates!")

    io = IO.IOClass()
    mover = Move.MoveClass()
    listener = Script.ScriptClass()
    tf = Transform.TransformClass()
    action_client = AmrActionClient()

    listener.wait_tm_connect()
    tf.add_vbases(vbase_pick, vbase_place)

    try:
        # while True:
            # mover.set_position(view_pick)
            # pick, safepick = get_positions(listener, pickplace_node, cli, tf, "vbase_pick", vjob_name)
            # mover.set_position(safepick)
            
            # io.open()
            # mover.set_position(pick)
            # io.close()
            # mover.set_position(safepick)

            # mover.set_position(view_place)
            # place, safeplace = get_positions(listener, pickplace_node, cli, tf, "vbase_place", vjob_name)
            # mover.set_position(safeplace)
            # mover.set_position(place)
            # io.open()
            # mover.set_position(safeplace)

        goal2result = action_client.send_goal('Goal2')

        mover.set_position(view_pick)
        pick, safepick = get_positions(listener, pickplace_node, cli, tf, "vbase_pick", vjob_name)
        mover.set_position(safepick)
        io.open()
        mover.set_position(pick)
        io.close()
        mover.set_position(safepick)

        goal1result = action_client.send_goal('Goal1')

        mover.set_position(view_place)
        place, safeplace = get_positions(listener, pickplace_node, cli, tf, "vbase_place", vjob_name)
        mover.set_position(safeplace)
        mover.set_position(place)
        io.open()
        mover.set_position(safeplace)

    except KeyboardInterrupt:
            pickplace_node.get_logger().info("Program shut down!")

    
    

if __name__ == '__main__':
    main()

