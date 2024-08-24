from setuptools import find_packages, setup

package_name = 'basic_pkg'

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
    maintainer='smkim',
    maintainer_email='smkim@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'helloworld_publisher = basic_pkg.helloworld_publisher:main',
            'helloworld_subscriber = basic_pkg.helloworld_subscriber:main'
        ],
    },
)
