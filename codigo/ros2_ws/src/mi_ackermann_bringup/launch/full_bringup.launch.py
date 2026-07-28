"""
full_bringup.launch.py
-----------------------
Levanta TODO el sistema de una vez:

  1. carlikebot.launch.xml
       -> ros2_control_node carga el plugin mi_ackermann_hardware/AckermannHardware,
          que en on_configure() abre la conexion TCP con el EV3 (ver
          ackermann_hardware_interface.cpp). NO hace falta un nodo aparte para
          "conectar" con el EV3: eso ya lo hace el controller_manager solo.
       -> robot_state_publisher (publica TF de todos los links, incluido
          base_link -> laser_frame)
       -> spawner de joint_state_broadcaster + bicycle_steering_controller
          (este ultimo publica la TF odom -> base_link porque
          enable_odom_tf: true en carlikebot_controllers.yaml)

  2. sllidar_ros2 (lidar C1) -> publica /scan en el frame "laser_frame"
     (tiene que coincidir con el nombre que pusimos en el xacro).

  3. slam_toolbox (async, modo mapping) con slam_params.yaml.

  4. teleop_twist_keyboard, en modo "stamped" (TwistStamped), remapeado
     directo a /bicycle_steering_controller/reference.

NOTA IMPORTANTE sobre el teclado:
  teleop_twist_keyboard necesita el foco del teclado en SU PROPIA terminal
  para leer las teclas (usa stdin en modo raw). Si lo lanzas en background
  dentro de un launch normal, el nodo arranca pero nunca va a "ver" las
  teclas que presionas en otra ventana. Por eso aqui se lanza con
  'xterm -e' para que abra su propia ventana de terminal.
  Si tu Raspberry no tiene entorno grafico / xterm, corre el teclado
  a mano en otra terminal (ver instrucciones al final del chat) en vez
  de usar el argumento use_keyboard:=true.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_share = get_package_share_directory('mi_ackermann_bringup')
    sllidar_share = get_package_share_directory('sllidar_ros2')

    # ---------------------------------------------------------------- args
    use_keyboard_arg = DeclareLaunchArgument(
        'use_keyboard', default_value='true',
        description='Lanzar teleop_twist_keyboard en una xterm aparte'
    )

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(bringup_share, 'config', 'slam_params.yaml'),
        description='Archivo de parametros de slam_toolbox'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Puerto serie del lidar C1'
    )

    # ---- TF base_link -> laser -----------------------------------------
    # use_manual_lidar_tf = true  -> este launch publica la TF a mano con
    #   static_transform_publisher (NO toques el xacro).
    # use_manual_lidar_tf = false -> se asume que ya agregaste el link
    #   'laser_frame' al xacro (mobile_base.xacro) y que
    #   robot_state_publisher es quien publica esa TF.
    # NUNCA los dos en true al mismo tiempo: dos nodos publicando la
    # misma transformacion (mismo parent+child) generan una TF que salta
    # entre un valor y otro, exactamente el mismo tipo de problema que
    # el doble publicador de odom->base_link que vimos en ev3_bridge.py.
    use_manual_lidar_tf_arg = DeclareLaunchArgument(
        'use_manual_lidar_tf', default_value='true',
        description='true = publica base_link->laser con static_transform_publisher '
                     'en vez de definirlo en el xacro'
    )

    frame_id_arg = DeclareLaunchArgument(
        'laser_frame_id', default_value='laser',
        description='Nombre del frame del lidar. Si usas la TF manual, este es '
                     'el child-frame-id del static_transform_publisher. Si usas '
                     'el xacro, pon aqui "laser_frame" (el nombre que le dimos '
                     'al link) y pasa use_manual_lidar_tf:=false.'
    )

    # valores medidos/ajustados a tu montaje real (mismos que probaste a mano,
    # pero ahora explicitos y con el orden correcto)
    tf_x_arg = DeclareLaunchArgument('lidar_tf_x', default_value='0.025')
    tf_y_arg = DeclareLaunchArgument('lidar_tf_y', default_value='0.06')
    tf_z_arg = DeclareLaunchArgument('lidar_tf_z', default_value='0.012')
    tf_roll_arg = DeclareLaunchArgument('lidar_tf_roll', default_value='0.0')
    tf_pitch_arg = DeclareLaunchArgument('lidar_tf_pitch', default_value='-0.087')
    tf_yaw_arg = DeclareLaunchArgument('lidar_tf_yaw', default_value='-1.5708')

    use_keyboard = LaunchConfiguration('use_keyboard')
    slam_params_file = LaunchConfiguration('slam_params_file')
    serial_port = LaunchConfiguration('serial_port')
    laser_frame_id = LaunchConfiguration('laser_frame_id')
    use_manual_lidar_tf = LaunchConfiguration('use_manual_lidar_tf')

    # ---------------------------------------------------------- 1) chasis
    carlikebot = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'carlikebot.launch.xml')
        )
    )

    # -------------------------------------------------- 1.5) TF manual lidar
    # Usa --x/--y/--z/--roll/--pitch/--yaw explicitos (no posicionales) para
    # que no se repita el mezclado yaw/roll que tuviste con el comando a mano.
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_tf',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('lidar_tf_x'),
            '--y', LaunchConfiguration('lidar_tf_y'),
            '--z', LaunchConfiguration('lidar_tf_z'),
            '--roll', LaunchConfiguration('lidar_tf_roll'),
            '--pitch', LaunchConfiguration('lidar_tf_pitch'),
            '--yaw', LaunchConfiguration('lidar_tf_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', laser_frame_id,
        ],
        condition=IfCondition(use_manual_lidar_tf),
    )

    # ---------------------------------------------------------- 2) lidar
    sllidar_c1 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(sllidar_share, 'launch', 'sllidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': serial_port,
            'frame_id': laser_frame_id,
            # ajusta si tu C1 queda "al reves" fisicamente:
            'inverted': 'false',
            'angle_compensate': 'true',
        }.items(),
    )

    # ---------------------------------------------------------- 3) slam
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
    )

    # ---------------------------------------------------------- 4) teclado
    # cmd_vel_topic remapeado directo al topico de referencia del
    # bicycle_steering_controller (TwistStamped).
    teleop_keyboard = ExecuteProcess(
        cmd=[
            'xterm', '-e',
            'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
            '--ros-args',
            '-p', 'stamped:=true',
            '-p', 'frame_id:=base_link',
            '-r', '/cmd_vel:=/bicycle_steering_controller/reference',
        ],
        output='screen',
        condition=IfCondition(use_keyboard),
    )

    return LaunchDescription([
        use_keyboard_arg,
        slam_params_arg,
        serial_port_arg,
        frame_id_arg,
        use_manual_lidar_tf_arg,
        tf_x_arg,
        tf_y_arg,
        tf_z_arg,
        tf_roll_arg,
        tf_pitch_arg,
        tf_yaw_arg,
        carlikebot,
        lidar_static_tf,
        sllidar_c1,
        slam_toolbox_node,
        teleop_keyboard,
    ])
