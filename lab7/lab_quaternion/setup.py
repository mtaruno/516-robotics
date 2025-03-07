from glob import glob
from setuptools import setup

package_name = 'lab_quaternion'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'pyquaternion'],
    zip_safe=True,
    maintainer='mtaruno',
    maintainer_email='mtaruno@uw.edu',
    description='ROS2 package for quaternion interpolation and trajectory planning',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_from_csv = lab_quaternion.trajectory_from_csv:main',
            'gen3lite_pymoveit2 = lab_quaternion.gen3lite_pymoveit2:main',
            'pick_and_place = lab_quaternion.pick_and_place:main',
        ],
    },
    data_files=[
        # This will ensure package.xml is installed
        ('share/' + package_name, ['package.xml']),
        # Install the launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Install the world files (SDF)
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
)

