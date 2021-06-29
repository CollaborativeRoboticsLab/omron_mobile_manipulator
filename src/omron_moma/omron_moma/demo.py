import rclpy
import time
import sys
import json
from ament_index_python.packages import get_package_share_directory
from rclpy import node
pp_share = get_package_share_directory('pickplace')
pp_library =  pp_share + '/pickplace/pp_library'

from pp_library import IO, Transform, Script, Move
from pickplace_msgs import AskModbus
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Bool

# Get the coordinates of the new vision base
def get_base(cli):
    while not cli.wait_for_service(timeout_sec=1.0):
        print("Service not available...")
    req = AskModbus.Request()
    req.req = 'get_base'
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(future)
    return future.result().position


# Get the new pick or place positions w.r.t the new vision base
def get_positions(listener, cli, tf, vbase_name, vjob_name):
    listener.exit_script()
    listener.change_base(vjob_name)
    time.sleep(0.1)
    new_vbase = get_base(cli)
    time.sleep(0.1)
    listener.change_base("RobotBase")
    time.sleep(0.1)
    if (vbase_name == "vbase_pick"):
        return tf.get_picks(new_vbase, vbase_name)
    elif (vbase_name == "vbase_place"):
        return tf.get_places(new_vbase, vbase_name)
    else:
        return new_vbase


# Sets the coordinates of the destination (pick/place) to a parameter in a 
# publisher, to be displayed in RViz
def call_set_parameters(node, coordinates):
    client = node.create_client(
        SetParameters,
        'destination_node/set_parameters'.format_map(locals()))
  
    ready = client.wait_for_service(timeout_sec=5.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')

    request = SetParameters.Request()
    param_values = ParameterValue(type = ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value = coordinates)
    request.parameters = [Parameter(name = 'destination_param', value = param_values)]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future) 

    response = future.result()
    if response is None:
        e = future.exception()
        raise RuntimeError(
            'Exception while calling service of node '
            "'{args.node_name}': {e}".format_map(locals()))
    return response



def main():
    rclpy.init()
    pickplace_node = rclpy.create_node('pickplace_node')
    flagpublisher = pickplace_node.create_publisher(Bool, 'objectflag', 10)
    cli = pickplace_node.create_client(AskModbus, 'ask_modbus')
    vjob_name = ""
    view_pick = []
    view_place = []
    vbase_pick = []
    vbase_place = []
    with open(pp_share + '/config.txt') as json_file:
        data = json.load(json_file)
        home_pos = data['home_pos']
        vjob_name = data['vjob_name']
        view_pick = data['view_pick']
        view_place = data['view_place']
        vbase_pick = data['vbase_pick']
        vbase_place = data['vbase_place']
    
    io = IO.IOClass()
    mover = Move.MoveClass()
    listener = Script.ScriptClass()
    tf = Transform.TransformClass()

    listener.wait_tm_connect()
    tf.add_vbases(vbase_pick, vbase_place)

    try:
        while True:
            msg = Bool()
            msg.data = False
            flagpublisher.publish(msg)
            mover.set_position(view_pick)
            pick, safepick = get_positions(listener, cli, tf, "vbase_pick", vjob_name)
            call_set_parameters(pickplace_node, pick)
            mover.set_position(safepick)
            
            io.open()
            mover.set_position(pick)
            msg.data = True
            flagpublisher.publish(msg)
            io.close()
            mover.set_position(safepick)

            mover.set_position(view_place)
            place, safeplace = get_positions(listener, cli, tf, "vbase_place", vjob_name)
            call_set_parameters(pickplace_node, place)
            mover.set_position(safeplace)
            mover.set_position(place)
            io.open()
            msg.data = False
            flagpublisher.publish(msg)
            mover.set_position(safeplace)

    except KeyboardInterrupt:
            pickplace_node.get_logger().info("Program shut down!")

    
    

if __name__ == '__main__':
    main()

