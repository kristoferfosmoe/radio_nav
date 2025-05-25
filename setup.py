from setuptools import find_packages, setup

package_name = 'radio_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/radio_nav_with_kalman.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial', 'pynmea2', 'numpy'],
    zip_safe=True,
    maintainer='kris',
    maintainer_email='kris@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "gps_node = radio_nav.gps_reader:main",
            "uwb_node = radio_nav.uwb_reader:main",
            "beacon_estimator = radio_nav.beacon_estimator:main",
            "display_nav = radio_nav.display_relnav:main",
            "gps_kalman_filter = radio_nav.gps_kalman_filter:main",
            "beacon_estimator_kalman = radio_nav.beacon_estimator_with_kalman:main"
        ],
    },
)
