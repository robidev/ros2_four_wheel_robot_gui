from setuptools import setup

package_name = 'kilted_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/kilted_gui.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='GUI control interface for the Kilted Robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'kilted_gui = kilted_gui.main:main',
        ],
    },
)
