from setuptools import setup
import os
from glob import glob


package_name = 'fpfh_classifier_aggregator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*launch.py')),
        (os.path.join('lib/python3.8/site-packages', package_name, "savedModel"), glob('savedModel/new/*.h5')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmed Abdullah',
    maintainer_email='ahmed.riyadh.abdullah@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'fpfh_sub_1 = fpfh_classifier_aggregator.fpfh_classifier_node_1:main',
                'fpfh_sub_2 = fpfh_classifier_aggregator.fpfh_classifier_node_2:main',
                'fpfh_sub_3 = fpfh_classifier_aggregator.fpfh_classifier_node_3:main',
                'fpfh_sub_4 = fpfh_classifier_aggregator.fpfh_classifier_node_4:main',
                'fpfh_sub_5 = fpfh_classifier_aggregator.fpfh_classifier_node_5:main',
                'fpfh_sub_6 = fpfh_classifier_aggregator.fpfh_classifier_node_6:main',
                'fpfh_sub_7 = fpfh_classifier_aggregator.fpfh_classifier_node_7:main',
                'fpfh_sub_8 = fpfh_classifier_aggregator.fpfh_classifier_node_8:main',
                'fpfh_sub_9 = fpfh_classifier_aggregator.fpfh_classifier_node_9:main',
                'fpfh_sub_10 = fpfh_classifier_aggregator.fpfh_classifier_node_10:main',
                'fpfh_sub_11 = fpfh_classifier_aggregator.fpfh_classifier_node_11:main',
                'fpfh_sub_12 = fpfh_classifier_aggregator.fpfh_classifier_node_12:main',
                'fpfh_sub_13 = fpfh_classifier_aggregator.fpfh_classifier_node_13:main',
                'fpfh_sub_14 = fpfh_classifier_aggregator.fpfh_classifier_node_14:main',
                'lstm_aggregator = fpfh_classifier_aggregator.lstm_aggregator:main',
        ],
    },
)
