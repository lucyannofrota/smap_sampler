import launch
import launch_ros.actions

def generate_launch_description():
    # launch_rviz = launch_ros.actions.Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    # )
    
    	
    # set_sim_time = launch_ros.actions.SetParameter(name='use_sim_time', value=True)

    node = launch_ros.actions.Node(
            package='smap_sampler',
            executable='smap_sampler_node',
            name='sampler_node',
            namespace='smap'
    )
    return launch.LaunchDescription([
        # set_sim_time,
        node
  ])