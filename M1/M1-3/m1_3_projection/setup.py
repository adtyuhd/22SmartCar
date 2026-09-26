from setuptools import find_packages, setup


package_name = 'm1_3_projection'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(
        exclude=['test']
    ),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='adtyuhd',
    maintainer_email='student@example.com',
    description=(
        'M1-3 camera and laser time synchronization, '
        'TF2 transformation and projection.'
    ),
    license='Apache-2.0',
    tests_require=[
        'pytest',
    ],
    entry_points={
        'console_scripts': [
            (
                'tf_static_publisher = '
                'm1_3_projection.tf_static_publisher:main'
            ),
            (
                'sync_test_node = '
                'm1_3_projection.sync_test_node:main'
            ),
            (
                'sync_sweep_node = '
                'm1_3_projection.sync_sweep_node:main'
            ),
        ],
    },
)