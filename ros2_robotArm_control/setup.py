from setuptools import setup

package_name = 'ros2_robotarm_control'

setup(
	name=package_name,
	version='0.1.0',
	packages=[package_name],
	data_files=[
		('share/ament_index/resource_index/packages',
			['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	author='sojung',
	author_email='sojung0320@gmail.com',
	maintainer='sojung',
	keywords=['ROS2','Detection','bluetooth'],
	description='Robot Arm Control',
	tests_require=['pytest'],
	entry_points={
		'console_scripts':[
			'client=ros2_robotarm_control.unityBluetooth:main',
			'service=ros2_robotarm_control.robotArm:main'
		]
	}
)
