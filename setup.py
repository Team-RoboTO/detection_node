from setuptools import setup

package_name = 'detection_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auto',
    maintainer_email='roboto.polito@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'talker = detection_node.webcam:main',
        	'detector = detection_node.detector:main',
            'v = detection_node.cv_view:main',
            'point_cloud_printer = detection_node.point_cloud_extractor:main',
            'pose_extractor = detection_node.pose_extractor:main',
            'serial = detection_node.serial_port:main'
        ],
    },
)
