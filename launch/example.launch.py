import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler, LogInfo, EmitEvent
from launch.events import Shutdown

def getFullFilePath(name, dir):
    ret = os.path.join(
        get_package_share_directory('pacsim'),
        dir,
        name
    )
    return ret


def generate_launch_description():
  track_name = "FSE23.yaml"
  track_frame = "map"
  realtime_ratio = 1.0
  discipline = "autocross"

  xacro_file_name = 'separate_model.xacro'
  xacro_path = getFullFilePath(xacro_file_name, "urdf")


  nodePacsim = Node(
          package='pacsim',
          namespace='pacsim',
          executable='pacsim_node',
          name='pacsim_node',
          parameters = [{'use_sim_time':True}, {"track_name" : getFullFilePath(track_name, "tracks")}, {"grip_map_path" : getFullFilePath("gripMap.yaml", "tracks")}, {"track_frame" : track_frame}, {"realtime_ratio" : realtime_ratio}, 
                        {"report_file_dir" : "/tmp"}, {"main_config_path" : getFullFilePath("mainConfig.yaml", dir="config")}, {"perception_config_path" : getFullFilePath("perception.yaml", dir="config")}, 
                        {"sensors_config_path" : getFullFilePath("sensors.yaml", dir="config")}, {"vehicle_model_config_path" : getFullFilePath("vehicleModel.yaml", dir="config")}, {"discipline" : discipline}],
          output="screen",
          emulate_tty=True)
  nodePacsimShutdownEventHandler = RegisterEventHandler(
        OnProcessExit(
            target_action=nodePacsim,
            on_exit=[
                LogInfo(msg=('Pacsim closed')),
                EmitEvent(event=Shutdown(
                    reason='Pacsim closed, shutdown whole launchfile')),
            ]
        )
  )
  robot_state_publisher= Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen',
          parameters=[{'use_sim_time':True}, {'publish_frequency':float(1000),}],
          arguments=[xacro_path])
  return LaunchDescription([nodePacsim, nodePacsimShutdownEventHandler, robot_state_publisher])