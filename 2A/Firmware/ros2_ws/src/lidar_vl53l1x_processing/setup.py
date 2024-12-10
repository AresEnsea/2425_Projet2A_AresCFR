from setuptools import find_packages, setup

package_name = 'lidar_vl53l1x_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mowibox',
    maintainer_email='ousmane.thg@gmail.com',
    description='STM32 TOF sensors VL53L1X lidar data processing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'lidar_ensea = lidar_vl53l1x_processing.data_processing:main',
        ],
    },
)
