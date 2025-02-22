from setuptools import setup

package_name = 'turtlebot_mover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=['turtlebot_mover.move_turtlebot_grid'],  # Correctly reference the script
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A script to move TurtleBot to grid locations',
    license='TODO: License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_turtlebot_grid = turtlebot_mover.move_turtlebot_grid:main',
        ],
    },
)

