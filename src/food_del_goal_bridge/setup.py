from setuptools import find_packages, setup

package_name = 'food_del_goal_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='panha',
    maintainer_email='dinsopheakpanha24@gmail.com',
    description='Goal bridge: /goal_pose → ComputePathToPose action → /plan',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'goal_bridge    = food_del_goal_bridge.goal_bridge:main',
            'path_recorder  = food_del_goal_bridge.path_recorder:main',
        ],
    },
)
