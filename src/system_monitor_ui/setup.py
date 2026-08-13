from setuptools import setup

package_name = 'system_monitor_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/templates',
            ['templates/smu.html', 'templates/db.html'],
        ),
    ],
    install_requires=['setuptools', 'flask', 'Pillow'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description=(
        'Web dashboard (live monitoring + DB run history) for the '
        'palletizing system, with an optional ROS2 bridge.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = system_monitor_ui.server:main',
            'launcher = system_monitor_ui.launcher:main',
        ],
    },
)
