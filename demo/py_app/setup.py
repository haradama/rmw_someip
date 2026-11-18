from setuptools import setup

package_name = 'rmw_someip_demo_py_app'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rmw_someip contributors',
    maintainer_email='kokushimusokanshin@gmail.com',
    description='Extern mode demo in Python: an rclpy node with no awareness of SOME/IP',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vehicle_node = rmw_someip_demo_py_app.vehicle_node:main',
        ],
    },
)
