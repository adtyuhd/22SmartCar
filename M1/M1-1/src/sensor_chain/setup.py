from setuptools import find_packages, setup

package_name = 'sensor_chain'

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
    maintainer='adtyuhd',
    maintainer_email='leilei_8@foxmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'node_a_sensor = sensor_chain.node_a_sensor:main',
            'node_b_filter = sensor_chain.node_b_filter:main',
            'node_c_alarm = sensor_chain.node_c_alarm:main',
        ],
    },
)
