from setuptools import setup

package_name = 'snake_capa_publisher'

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
    maintainer='Wenpeng Wang',
    maintainer_email='wwang153@jhu.edu',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'capa_publisher = snake_capa_publisher.publisher_capa_sensor:main',
        	'capa_mapper = snake_capa_publisher.jointstate_publisher_capa_mapper:main',
        ],
    },
)
