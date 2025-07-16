from setuptools import setup

package_name = 'motor_commander'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lizihan',
    maintainer_email='2368521462@qq.com',
    description='上位机电机指令发布包',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander = motor_commander.motor_commander_node:main',
        ],
    },
)