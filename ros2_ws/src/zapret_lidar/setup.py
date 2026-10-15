from setuptools import setup

package_name = "zapret_lidar"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/lidar.launch.py"]),
        ("share/" + package_name + "/config", ["config/lidar.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="danilka-revin",
    maintainer_email="revin.d4n@gmail.com",
    description="Супервизор драйвера лидара для робота Zapret (ROS 2)",
    license="MIT",
    entry_points={
        "console_scripts": [
            # Узел супервизора: ros2 run zapret_lidar supervisor_node
            "supervisor_node = zapret_lidar.supervisor:main",
            # Диагностика без запуска узла:
            #   ros2 run zapret_lidar lidar_plan --print-plan
            #   ros2 run zapret_lidar lidar_plan --probe
            "lidar_plan = zapret_lidar.supervisor:main",
        ],
    },
)
