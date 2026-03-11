from setuptools import find_packages, setup

package_name = 'Simulation'

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
    description='Simulation tools',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'kinematic_sim = Simulation.kinematic_sim:main',
        ],
    },
)