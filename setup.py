from setuptools import setup

package_name = 'ndi_polaris_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joshua Liu',
    maintainer_email='liushuya7@gmail.com',
    description='A simple ROS2 Package for NDI Polaris Tracker',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'polaris_tracker = ndi_polaris_tracker.polaris_tracker:main'
        ],
    },
)
