from setuptools import find_packages, setup


package_name = 'warehouse_robot_missions'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='anastasios',

    maintainer_email='anastasios@todo.todo',

    description=(
        'Mission management package for '
        'the warehouse mobile robot.'
    ),

    license='TODO',

    tests_require=[
        'pytest'
    ],

    entry_points={
        'console_scripts': [
            (
                'warehouse_mission = '
                'warehouse_robot_missions.'
                'warehouse_mission:main'
            ),
            (
                'pose_recorder = '
                'warehouse_robot_missions.'
                'pose_recorder:main'
            ),
            (
                'mission_dashboard = '
                'warehouse_robot_missions.'
                'mission_dashboard:main'
            ),
        ],
    },
)
