import os
from glob import glob
from setuptools import setup

package_name = 'dummy_robot_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # # Include all conf files.
        (os.path.join('share', package_name, 'conf'), glob(os.path.join('conf', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabrielbermudez',
    maintainer_email='gabrielbermudez@gmail.com',
    description='Package for test',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
            'console_scripts': [
                    'talker = dummy_robot_test.publisher_member_function:main',
                    'listener = dummy_robot_test.subscriber_member_function:main',
                    'one_joint_state_publisher = dummy_robot_test.one_joint_state_publisher:main',
                    'dummy_joint_states = dummy_robot_test.dummy_joint_states:main',
            ],
    },
)