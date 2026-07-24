import launch
import launch_ros

from ament_index_python import get_package_share_directory

def generate_launch_description():
    urdf_tutorial_path=get_package_share_directory("fish_description")
    default_model_path=urdf_tutorial_path+"/urdf/first_robot.urdf";
    
    action_declare_arg_mode_path=launch.actions.DeclareLaunchArgument(
        name="model",default_value=str(default_model_path),
        description="URDF path"
    )

    robot_description=launch_ros.parameter_descriptions.Parametervalue(
        launch.substitutions.Command(
            ["cat",launch.substitutions.launch_configuration("model")]
        ),value_type=str
    )

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)