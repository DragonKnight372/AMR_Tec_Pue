from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'sensores_kalman'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),   # AGREGAR ESTO
         glob('rviz/*.rviz')),                           # AGREGAR ESTO
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abrahammh19 marml28',
    maintainer_email='abrahammorohdez@gmail.com',
    description='TODO: Package description',
    license='Apache 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'amr_encoder = sensores_kalman.amr_encoder_amh19:main',
        'amr_imu_encoder = sensores_kalman.amr_imu_encoder:main',
        'amr_pure_pursuit = sensores_kalman.amr_pure_pursuit:main',
        'fusion_kalman_node = sensores_kalman.fusion_kalman_node:main',
        'imu_kalman_all_node = sensores_kalman.imu_kalman_all_node:main',
        'lidar_kalman_node = sensores_kalman.lidar_kalman_node:main',
        'pose_ekf_amr = sensores_kalman.pose_ekf_amr:main',
        'pose_ekf_qcar_2 = sensores_kalman.pose_ekf_qcar_2:main',
        'pose_final_qcar = sensores_kalman.pose_final_qcar:main',
        'qcar_lidar_alert_2 = sensores_kalman.qcar_lidar_alert_2:main',
        'qcar_pose = sensores_kalman.qcar_pose:main',
        'pure_pursuit_node= sensores_kalman.qcar_pure_pursuit:main',
        'qcar_watchdog_node= sensores_kalman.qcar_watchdog_node:main',
        'trayectoria_grabar_csv= sensores_kalman.trayectoria_grabar_csv:main',
        'velocity_kalman_node = sensores_kalman.velocity_kalman_node:main'
        ],
},
)



