from setuptools import setup

package_name = 'lars_groundbot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tanayrs',
    maintainer_email='your_email@example.com',
    description='Control package for the lars_groundbot robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = lars_groundbot_control.control_node:main',
        ],
    },
)

