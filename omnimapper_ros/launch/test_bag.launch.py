import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

import launch
import launch.actions
import launch_ros.actions

def generate_launch_description():

	default_config = os.path.join(get_package_share_directory('omnimapper_ros'),
    'config', 'plane_test.yaml')
	print(default_config)

	start_omnimapper_ros_node = launch_ros.actions.Node(
		package='omnimapper_ros',
		node_executable='omnimapper_ros_node',
		node_name='omnimapper_ros_node',
		parameters=[{
			'use_sim_time': True,
			'use_planes': True,
			'use_icp': True,
			'use_objects': False,
			'use_occ_edge_icp': True,
			'use_tf': True,
			'use_error_plugin': False,
			'use_tsdf_plugin': False,
			'use_no_motion': False,
			'use_rgbd_sensor_base_tf_functor': False,
			'use_distortion_model': False,
			'odom_frame_name': "/world",
			'base_frame_name': "camera_link",
			'use_init_pose': False,
			'init_pose_from_tf': False,
			'cloud_topic_name': "/camera/pointcloud",
			'icp_leaf_size': 0.025,
			'icp_max_correspondence_distance': 0.1,
			'icp_trans_noise': 10.05,
			'icp_rot_noise': 10.05,
			'occ_edge_trans_noise': 0.05,
			'occ_edge_rot_noise': 0.1,
			'occ_edge_max_correspondence_dist': 0.05,
			'occ_edge_score_thresh': 0.1,
			'occ_edge_add_identity_on_fail': False,
			'tf_trans_noise': 0.05,
			'tf_rot_noise': 0.0872664626,
			'draw_pose_array': False,
			'add_pose_per_cloud': False,
			'evaluation_mode': False
		}],
		output='screen'
		
	)

	ld = launch.LaunchDescription()
	ld.add_action(start_omnimapper_ros_node)
	
	return ld