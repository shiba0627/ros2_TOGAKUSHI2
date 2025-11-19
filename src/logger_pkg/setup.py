from setuptools import find_packages, setup

package_name = 'logger_pkg'

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
    maintainer='ubuntu',
    maintainer_email='1e8ec3@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'compress_node = logger_pkg.compress_node:main',
            'joy_logger = logger_pkg.joy_logger:main',
            'image_logger = logger_pkg.image_logger:main',
            'obstacle_logger = logger_pkg.obstacle_logger:main',
            'odom_logger = logger_pkg.odom_logger:main',
        ],
    },
)
