from setuptools import setup

package_name = 'py_rrt_1'

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
    maintainer='Rachanon',
    maintainer_email='rachanon001@e.ntu.edu.sg',
    description='MLDA RRT Problem Trial 1 Python Part',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt_vanilla = py_rrt_1.py_rrt_vanilla_planner_node:main',
            'map_read = py_rrt_1.py_cv_window:main',
            'rrt_planner = py_rrt_1.py_rrt_planner_new_node:main',
            'map_painter = py_rrt_1.py_map_paint_node:main',
        ],
    },
)
