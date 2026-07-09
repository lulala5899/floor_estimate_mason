from setuptools import find_packages, setup
from glob import glob
from os.path import isfile

package_name = 'serial_to_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch',
            [path for path in glob('launch/*') if isfile(path)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='witsir',
    maintainer_email='zhangyh12024@shanghaitech.edu.cn',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_serial_baro = serial_to_ros2.esp32_serial_baro:main'
        ],
    },
)
