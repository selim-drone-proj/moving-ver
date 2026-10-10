from glob import glob

from setuptools import find_packages, setup

package_name = "odor_drone"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],   # Pi 에서는 추가로 smbus2, bme680 (README)
    zip_safe=True,
    maintainer="Drone_Stink team, selim-drone-proj",
    maintainer_email="dev@example.com",
    description="악취 드론 온보드 노드",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fc_state_node = odor_drone.fc_state_node:main",
            "gas_i2c_node = odor_drone.gas_i2c_node:main",
            "gas_sensor_node = odor_drone.gas_sensor_node:main",
            "odor_sample_node = odor_drone.odor_sample_node:main",
            "csv_logger_node = odor_drone.csv_logger_node:main",
            "fake_fc_node = odor_drone.fake_fc_node:main",
            "fake_gas_node = odor_drone.fake_gas_node:main",
        ],
    },
)
