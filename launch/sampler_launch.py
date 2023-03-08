import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='smap_sampler',
            executable='smap_sampler_node.cpp',
            name='smap_sampler_node'),
  ])