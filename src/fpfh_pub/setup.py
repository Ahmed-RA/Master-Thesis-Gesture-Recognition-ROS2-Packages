from setuptools import setup

package_name = 'fpfh_pub'

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
    maintainer='fdai6764',
    maintainer_email='ahmed.riyadh.abdullah@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fpfh_publisher = fpfh_pub.fpfh_publisher:main'
        ],
    },
)
