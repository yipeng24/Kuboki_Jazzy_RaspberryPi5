from setuptools import find_packages, setup

package_name = 'realsense_d435i_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/realsense_d435i.launch.py']),
        ('share/' + package_name + '/config', ['config/conf_realsense.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yipeng',
    maintainer_email='gouyipeng24@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
