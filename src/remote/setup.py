from setuptools import find_packages, setup

package_name = 'remote'

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
    maintainer='Ardith Ho',
    maintainer_email='ardithhowaiman@gmail.com',
    description='Remote nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "display_node = remote.display_node:main",
            "gpu_node = remote.gpu_node:main"
        ],
    },
)