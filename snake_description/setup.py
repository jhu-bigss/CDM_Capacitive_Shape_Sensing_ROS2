import os
from glob import glob
from setuptools import setup

package_name = 'snake_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=['snake_bend_gui'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all other files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joshua Liu',
    maintainer_email='liushuya7@gmail.com',
    description='snake tool ROS2 description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'snake_bend_slider_gui = snake_bend_gui.snake_bend_slider_gui:main',
        ],
    },
)
