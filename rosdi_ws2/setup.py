from setuptools import setup
from setuptools import find_packages

package_name = 'rosdi_ws2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'rodi_transport',
        'http_rodi_transport',
        'socket_rodi_transport',
        'rosdi_ws',
        'rodi_api',
        'rodi_node',
        'examples/obstacles_avoider',
        'overridable_action',
        ],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['examples/obstacles_avoider_launcher.py']),
    ],
    install_requires=['setuptools'],
    author='Jon Lucas',
    author_email='jonlucas@ekuthon.com',
    maintainer='Jon Lucas',
    maintainer_email='jonlucas@ekuthon.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License 2.0',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='RosDI interface.',
    license='Apache License 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'rosdi_ws = rosdi_ws:main',
            'obstacles_avoider = examples.obstacles_avoider:main',
            'launch = examples.obstacles_avoider_launcher:main',
        ],
    },
)
