from setuptools import find_packages, setup

package_name = 'manipulator'

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
    maintainer='mscrobotics2425laptop16',
    maintainer_email='1394698319@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ServerOfPerceptionAndGrasp = manipulator.server_of_perception_for_grasp:main',
            'TEST_ServerOfPerceptionAndGrasp = manipulator.pixel_to_base_verification:main',
        ],
    },
)
