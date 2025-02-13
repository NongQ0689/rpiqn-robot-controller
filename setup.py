from setuptools import find_packages, setup

package_name = 'rpiqn_robot_controller'

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
    maintainer='q',
    maintainer_email='q@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'myodom_node = rpiqn_robot_controller.myodom3:main',
            'ocgm_node = rpiqn_robot_controller.ocgm:main',
            'show_ocgm_node = rpiqn_robot_controller.showocgm:main',
            'bug0_node = rpiqn_robot_controller.bug0:main',
            'path_planing_node = rpiqn_robot_controller.path_planing6:main',
            'main_node = rpiqn_robot_controller.mainnode3:main',
        ],
    },
)
