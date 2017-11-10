from setuptools import setup

setup(
    name='rosdi_ws2',
    version='0.0.0',
    packages=[],
    py_modules=['rosdi_ws', 'remote_rodi_api'],
    install_requires=['setuptools'],
    author='Jon Lucas',
    author_email='jonlucas@ekuthon.com',
    maintainer='Jon Lucas',
    maintainer_email='jonlucas@ekuthon.com',
    keywords=['ROS'],
    description='RosDI interface.',
    license='BSD',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'rosdi_ws = rosdi_ws:main',
        ],
    },
)
