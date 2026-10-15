# Copyright 2026 Zapret robot control
#
# Запуск лидара робота Zapret:
#
#   ros2 launch zapret_lidar lidar.launch.py                       # всё сам
#   ros2 launch zapret_lidar lidar.launch.py model:=lds01 port:=/dev/ttyUSB0
#   ros2 launch zapret_lidar lidar.launch.py autostart:=false      # ждать ~/rescan
#
# По умолчанию поднимается наш супервизор (zapret_lidar/lidar_supervisor): он сам
# выбирает ROS 2-драйвер под подключённый лидар, запускает его и следит за
# свежестью /scan, а состояние публикует в /zapret/lidar/status — по нему
# веб-пульт и бортовой экран показывают, почему скана нет.
#
# Если нужно запустить драйвер вендора напрямую (отладка «как в документации»),
# задайте driver_package и driver_launch:
#
#   ros2 launch zapret_lidar lidar.launch.py \
#       driver_package:=rplidar_ros driver_launch:=rplidar_a1_launch.py \
#       port:=/dev/ttyUSB0
#
# Самый простой путь на роботе — ./scripts/ros2_lidar.sh: он собирает рабочее
# пространство, берёт порт и модель из .env и печатает статус.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _vendor_launch(context):
    """Включает launch-файл драйвера вендора, если он задан вручную."""
    package = context.perform_substitution(LaunchConfiguration("driver_package"))
    name = context.perform_substitution(LaunchConfiguration("driver_launch"))
    if not name:
        return []
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(package)
    except Exception as exc:  # noqa: BLE001 — пакет не поставлен
        return [LogInfo(msg="драйвер %s не найден (%s): поставь его командой "
                            "./scripts/ros2_setup.sh (поставит драйверы сам) или убери driver_launch"
                            % (package, exc))]
    return [LogInfo(msg="запускаю драйвер вендора %s/%s" % (package, name)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource("%s/launch/%s" % (share, name)))]


def generate_launch_description():
    model = LaunchConfiguration("model")
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    scan_topic = LaunchConfiguration("scan_topic")
    status_topic = LaunchConfiguration("status_topic")
    frame_id = LaunchConfiguration("frame_id")
    autostart = LaunchConfiguration("autostart")
    autoprobe = LaunchConfiguration("autoprobe")

    # Вендорский launch и наш супервизор вместе не запускаются: они боролись бы
    # за один последовательный порт лидара.
    use_vendor = PythonExpression(["'", LaunchConfiguration("driver_launch"), "' != ''"])

    supervisor = Node(
        package="zapret_lidar",
        executable="supervisor_node",
        name="lidar_supervisor",
        output="screen",
        condition=UnlessCondition(use_vendor),
        parameters=[{
            "model": model,
            "port": port,
            "baud": baud,
            "scan_topic": scan_topic,
            "status_topic": status_topic,
            "frame_id": frame_id,
            "autostart": autostart,
            "autoprobe": autoprobe,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="auto",
                              description="auto | lds01 | rplidar | ld06 | ld19 | stl27l | opencr"),
        DeclareLaunchArgument("port", default_value="",
                              description="порт(ы) лидара; пусто — автопоиск по /dev"),
        DeclareLaunchArgument("baud", default_value="0",
                              description="скорость порта; 0 — по умолчанию драйвера"),
        DeclareLaunchArgument("scan_topic", default_value="/scan",
                              description="топик sensor_msgs/LaserScan"),
        DeclareLaunchArgument("status_topic", default_value="/zapret/lidar/status",
                              description="топик состояния супервизора (JSON)"),
        DeclareLaunchArgument("frame_id", default_value="base_scan",
                              description="frame_id скана"),
        DeclareLaunchArgument("autostart", default_value="true",
                              description="запускать драйвер сразу; false — ждать ~/rescan"),
        DeclareLaunchArgument("autoprobe", default_value="true",
                              description="быстро проверять порты перед запуском драйвера"),
        DeclareLaunchArgument("driver_package", default_value="",
                              description="запустить launch вендора из этого пакета"),
        DeclareLaunchArgument("driver_launch", default_value="",
                              description="имя launch-файла вендора (например rplidar_a1_launch.py)"),
        OpaqueFunction(function=_vendor_launch),
        supervisor,
    ])
