from setuptools import find_packages, setup

package_name = 'creeper_cycle'

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
    maintainer='porweks',
    maintainer_email='porweks@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'cycle = creeper_cycle.creeper_cycle:main',
        'flower = creeper_cycle.creeper_movement:main',
        ],
    },
)
