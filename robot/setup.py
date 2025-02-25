from setuptools import find_packages, setup

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/_launch.py']),
        ('share/' + package_name + '/launch', ['launch/_launchTecl.py']),
        ('share/' + package_name + '/launch', ['launch/_launchTeclSim.py']),
        ('share/' + package_name + '/launch', ['launch/_launchSim.py']),
        ('share/' + package_name + '/launch', ['launch/_launchProg.py']),
        ('share/' + package_name + '/launch', ['launch/_launchProgSim.py']),
        ('share/' + package_name + '/launch', ['launch/_launchGazebo.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oswaldo',
    maintainer_email='oswaldo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "draw_circle = robot.draw_circle:main",
            "motores = robot.motores:main",
            "mover = robot.mover:main",
            "cinematica = robot.cinematica:main",
            "programar = robot.programar:main",
            "manipulador = robot.manipulador:main",
            "manipuladorTeclado = robot.manipuladorTeclado:main",
            "TestNode = robot.TestNode:main",
            "datos_sim = robot.datos_sim:main",
            "datos_simG = robot.datos_simG:main",
            "gazebo = robot.gazebo:main",
        ],
    },
)
