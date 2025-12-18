from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue # newly imported

def generate_launch_description():
    description_pkg = "arm_urdf"
    bringup_pkg = "controller"

    urdf_file = PathJoinSubstitution([
        FindPackageShare(description_pkg),
        "urdf",
        "testArmURDF.urdf"  # change to your new URDF filename
    ])

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare(bringup_pkg),
        "config",
        "arm_controllers.yaml"
    ])

    robot_description = {
    	"robot_description": ParameterValue(
    		Command(["cat ", urdf_file]),
    		value_type=str
    	)
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_yaml],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

    return LaunchDescription([rsp, control_node, jsb_spawner, arm_spawner, rviz])

