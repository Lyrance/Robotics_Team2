from setuptools import find_packages, setup

package_name = 'Object_Detection'

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
    maintainer='mscrobotics2425laptop21',
    maintainer_email='yuliang.li-3@postgrad.manchester.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collect_data =''Object_Detection.collect_data:main',
            'predict =''Object_Detection.predict :main',
            'test =''Object_Detection.test:main',
        ],
    },
)
