import os
from setuptools import find_packages, setup

package_name = 'py_serial_receive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), [
            'py_serial_receive/config/uart_format.json'  
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alex@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rf_receiver_node = py_serial_receive.rf_receiver_node:main',
        ],
    },
    # 指定使用的 Python 解釋器
    options={
        'build_scripts': {
            'executable': '/home/alex/miniconda3/envs/ros_env/bin/python',
        },
    },
)
