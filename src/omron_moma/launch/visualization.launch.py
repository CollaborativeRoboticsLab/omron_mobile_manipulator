import sys
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

pp_share = get_package_share_directory('pickplace')
pp_library =  pp_share + '/pickplace/pp_library'

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    args = []
    length = len(sys.argv)
    if (len(sys.argv) >= 5):
        i = 4
        while i < len(sys.argv):
            args.append(sys.argv[i])
            i = i + 1

    robot_description_config = load_file('omron_moma', 'MoMa_TM12.urdf')
    #robot_description_config = load_file('amr_visualisation', 'urdf/AMR_Platform.urdf')
    robot_description = {'robot_description' : robot_description_config}

    vis_config = get_package_share_directory('amr_visualisation') + "/param/vis_param.yaml"

    # RViz
    rviz_config_file = "src/omron_moma/config/moma_rviz.rviz"
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )
        
    # Joints Publisher
    joints_publisher_node = Node(
        package='amr_visualisation',
        executable='joints_publisher',
        output='screen',
    )
    
    # Publish Robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='log',
        parameters=[robot_description],
    )
    
    # Static TF
    static_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='pose_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'pose', 'world']
    )
    
    # Data Points Marker
    data_points_node = Node(
        package='amr_visualisation',
        executable='data_points_marker',
        output='screen',
        parameters=[vis_config],
    )
        
    # Goals Marker
    goals_node = Node(
        package='amr_visualisation',
        executable='goals_marker',
        output='screen',
    )
    
    marker_publisher_node = Node(
        package='pp_marker',
        executable='marker',
        output='screen'
    )

    return LaunchDescription([
    	rviz_node,
        robot_state_publisher,
        joints_publisher_node,
        static_world,
        goals_node,
        marker_publisher_node,
        data_points_node
        ])

