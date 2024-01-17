from setuptools import find_packages, setup, os
import glob

package_name = 'imu_package'
submodules = 'imu_package/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name), glob.glob('launch/*launch.[pxy][yma]*')),
	(os.path.join('share', package_name,'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='crubs',
    maintainer_email='crubs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'imu_main = imu_package.imu_main:main',
        'ekf_listener = imu_package.ekf_listener:main',
        'position_zero = imu_package.position_zero:main',
        'gps_listener = imu_package.gps_listener:main'
        ],
    },
)
