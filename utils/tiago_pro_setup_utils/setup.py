from setuptools import find_packages, setup

package_name = 'tiago_pro_setup_utils'

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
    maintainer='kalman',
    maintainer_email='samuele.sandrini@polito.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'table_scene_publisher_node = tiago_pro_setup_utils.table_scene_publisher_node:main',
          'static_tip_frame_publisher_node = tiago_pro_setup_utils.static_tip_frame_publisher_node:main',
          'set_param_once_node = tiago_pro_setup_utils.set_param_once_node:main',
        ],
    },
)
