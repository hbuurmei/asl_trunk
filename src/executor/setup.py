from setuptools import find_packages, setup

package_name = 'executor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/executor', ['executor/executor_node.py']),
        ('share/' + package_name + '/executor', ['executor/data_collection_node.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asl',
    maintainer_email='asl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'executor_node = executor.executor_node:main',
            'data_collection_node = executor.data_collection_node:main',
        ],
    },
)
