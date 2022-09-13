from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(package='fpfh_classifier_aggregator',
             executable='fpfh_sub_1',
             name='fpfh_sub_1',
             parameters=[
                 {'cycles': 14},
             ],
             ),
        Node(package='fpfh_classifier_aggregator',
             executable='fpfh_sub_2',
             name='fpfh_sub_2',
             parameters=[
                 {'cycles': 14},
             ],
             ),
        Node(package='fpfh_classifier_aggregator',
             executable='fpfh_sub_3',
             name='fpfh_sub_3',
             parameters=[
                 {'cycles': 14},
             ],
             ),
        Node(package='fpfh_classifier_aggregator',
             executable='fpfh_sub_4',
             name='fpfh_sub_4',
             parameters=[
                 {'cycles': 14},
             ],
             ),
        Node(package='fpfh_classifier_aggregator',
             executable='fpfh_sub_5',
             name='fpfh_sub_5',
             parameters=[
                 {'cycles': 14},
             ],
             ),
        Node(package='fpfh_classifier_aggregator',
             executable='fpfh_sub_6',
             name='fpfh_sub_6',
             parameters=[
                 {'cycles': 14},
             ],
             ),
        Node(package='fpfh_classifier_aggregator',
             executable='fpfh_sub_7',
             name='fpfh_sub_7',
             parameters=[
                 {'cycles': 14},
             ],
             ),
        Node(package='fpfh_classifier_aggregator',
             executable='fpfh_sub_8',
             name='fpfh_sub_8',
             parameters=[
                 {'cycles': 14},
             ],
             ),
        Node(package='fpfh_classifier_aggregator',
             executable='fpfh_sub_9',
             name='fpfh_sub_9',
             parameters=[
                 {'cycles': 14},
             ],
             ),
        Node(package='fpfh_classifier_aggregator',
             executable='fpfh_sub_10',
             name='fpfh_sub_10',
             parameters=[
                 {'cycles': 14},
             ],
             ),
        Node(package='fpfh_classifier_aggregator',
             executable='fpfh_sub_11',
             name='fpfh_sub_11',
             parameters=[
                 {'cycles': 14},
             ],
             ),
        Node(package='fpfh_classifier_aggregator',
             executable='fpfh_sub_12',
             name='fpfh_sub_12',
             parameters=[
                 {'cycles': 14},
             ],
             ),
        Node(package='fpfh_classifier_aggregator',
             executable='fpfh_sub_13',
             name='fpfh_sub_13',
             parameters=[
                 {'cycles': 14},
             ],
             ),
        Node(package='fpfh_classifier_aggregator',
             executable='fpfh_sub_14',
             name='fpfh_sub_14',
             parameters=[
                 {'cycles': 14},
             ],
             ),     
    ])
