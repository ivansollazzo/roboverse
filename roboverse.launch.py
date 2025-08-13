import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    current_dir = os.path.dirname(os.path.realpath(__file__))
    endpoint_launch_path = os.path.join(
        current_dir, 'src', 'ros_tcp_endpoint', 'launch', 'endpoint.py'
    )

    # Load the target places
    target_places_file = os.path.join(
        current_dir,
        'src',
        'unicycle',
        'config',
        'target_places.yaml'
    )

    # Load the rendez-vous places
    rv_places_file = os.path.join(
        current_dir,
        'src',
        'unicycle',
        'config',
        'rv_places.yaml'
    )

    # Create dictionaries to manage places
    target_places = {}
    rv_places = {}

    # Load the target places
    with open(target_places_file, 'r') as f:
        target_places = yaml.safe_load(f)

    # Load the rendez-vous places
    with open(rv_places_file, 'r') as f:
        rv_places = yaml.safe_load(f)

    return LaunchDescription([
        
        # INCLUDE ROS TCP CONNECTOR ENDPOINT LAUNCH
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(endpoint_launch_path)
        ),

        ## UNICYCLE 0 NODES
        
        # Kinematics Controller
        Node(
            package='unicycle',
            executable='kinematics_controller',
            name='unicycle_0_kinematics_controller',
            parameters=[{'unicycle_id': 'unicycle_0'}],
            output='screen'
        ),
        
        # Task Manager
        Node(
            package='unicycle',
            executable='task_manager',
            name='unicycle_0_task_manager',
            parameters=[
                {'unicycle_id': 'unicycle_0'},
                {'num_unicycles': 3},
                {'unicycle_number': 0},
                {'num_places': 3},
                {'target_place_0_z': target_places['target_place_0']['z']},
                {'target_place_0_x': target_places['target_place_0']['x']},
                {'target_place_1_z': target_places['target_place_1']['z']},
                {'target_place_1_x': target_places['target_place_1']['x']},
                {'target_place_2_z': target_places['target_place_2']['z']},
                {'target_place_2_x': target_places['target_place_2']['x']},
                {'rv_desired_place_z': rv_places['desired_place_0']['z']},
                {'rv_desired_place_x': rv_places['desired_place_0']['x']},
                {'current_target_place': 0}
            ],
            output='screen'
        ),
        # Rendez-vous Manager
        Node(
            package='unicycle',
            executable='rendezvous_manager',
            name='unicycle_0_rendezvous_manager',
            parameters=[
                {'unicycle_id': 'unicycle_0'},
                {'num_unicycles': 3},
                {'unicycle_number': 0}
            ],
            output='screen'
        ),
        # Knowledge Register
        Node(
            package='unicycle',
            executable='knowledge_register',
            name='unicycle_0_knowledge_register',
            parameters=[
                {'unicycle_id': 'unicycle_0'}
            ],
            output='screen'
        ),
        # Sensors Manager
        Node(
            package='unicycle',
            executable='sensors_manager',
            name='unicycle_0_sensors_manager',
            parameters=[
                {'unicycle_id': 'unicycle_0'},
                {'unicycle_number': 0}
            ]
        ),

        ## UNICYCLE 1 NODES
        # Kinematics Controller
        Node(
            package='unicycle',
            executable='kinematics_controller',
            name='unicycle_1_kinematics_controller',
            parameters=[{'unicycle_id': 'unicycle_1'}],
            output='screen'
        ),
        
        # Task Manager
        Node(
            package='unicycle',
            executable='task_manager',
            name='unicycle_1_task_manager',
            parameters=[
                {'unicycle_id': 'unicycle_1'},
                {'num_unicycles': 3},
                {'unicycle_number': 1},
                {'num_places': 3},
                {'target_place_0_z': target_places['target_place_0']['z']},
                {'target_place_0_x': target_places['target_place_0']['x']},
                {'target_place_1_z': target_places['target_place_1']['z']},
                {'target_place_1_x': target_places['target_place_1']['x']},
                {'target_place_2_z': target_places['target_place_2']['z']},
                {'target_place_2_x': target_places['target_place_2']['x']},
                {'rv_desired_place_z': rv_places['desired_place_1']['z']},
                {'rv_desired_place_x': rv_places['desired_place_1']['x']},
                {'current_target_place': 1}
            ],
            output='screen'
        ),
        # Rendez-vous Manager
        Node(
            package='unicycle',
            executable='rendezvous_manager',
            name='unicycle_1_rendezvous_manager',
            parameters=[
                {'unicycle_id': 'unicycle_1'},
                {'num_unicycles': 3},
                {'unicycle_number': 1}
            ],
            output='screen'
        ),
        # Knowledge Register
        Node(
            package='unicycle',
            executable='knowledge_register',
            name='unicycle_1_knowledge_register',
            parameters=[
                {'unicycle_id': 'unicycle_1'}
            ],
            output='screen'
        ),
        # Sensors Manager
        Node(
            package='unicycle',
            executable='sensors_manager',
            name='unicycle_1_sensors_manager',
            parameters=[
                {'unicycle_id': 'unicycle_1'},
                {'unicycle_number': 1}
            ],
            output='screen'
        ),

        ## UNICYCLE 2 NODES
        # Kinematics Controller
        Node(
            package='unicycle',
            executable='kinematics_controller',
            name='unicycle_2_kinematics_controller',
            parameters=[{'unicycle_id': 'unicycle_2'}],
            output='screen'
        ),

        # Task Manager
        Node(
            package='unicycle',
            executable='task_manager',
            name='unicycle_2_task_manager',
            parameters=[
                {'unicycle_id': 'unicycle_2'},
                {'num_unicycles': 3},
                {'unicycle_number': 2},
                {'num_places': 3},
                {'target_place_0_z': target_places['target_place_0']['z']},
                {'target_place_0_x': target_places['target_place_0']['x']},
                {'target_place_1_z': target_places['target_place_1']['z']},
                {'target_place_1_x': target_places['target_place_1']['x']},
                {'target_place_2_z': target_places['target_place_2']['z']},
                {'target_place_2_x': target_places['target_place_2']['x']},
                {'rv_desired_place_z': rv_places['desired_place_2']['z']},
                {'rv_desired_place_x': rv_places['desired_place_2']['x']},
                {'current_target_place': 2}
            ],
            output='screen'
        ),
        # Rendez-vous Manager
        Node(
            package='unicycle',
            executable='rendezvous_manager',
            name='unicycle_2_rendezvous_manager',
            parameters=[
                {'unicycle_id': 'unicycle_2'},
                {'num_unicycles': 3},
                {'unicycle_number': 2}
            ],
            output='screen'
        ),
        # Knowledge Register
        Node(
            package='unicycle',
            executable='knowledge_register',
            name='unicycle_2_knowledge_register',
            parameters=[
                {'unicycle_id': 'unicycle_2'}
            ],
            output='screen'
        ),
        # Sensors Manager
        Node(
            package='unicycle',
            executable='sensors_manager',
            name='unicycle_2_sensors_manager',
            parameters=[
                {'unicycle_id': 'unicycle_2'},
                {'unicycle_number': 2}
            ],
            output='screen'
        )

    ])