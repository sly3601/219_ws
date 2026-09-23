from setuptools import find_packages, setup

package_name = 'tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
        ('share/' + package_name + '/launch', [
            'launch/joint_tuner.launch.py',
            'launch/joint_state_viewer.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sly',
    maintainer_email='sly@example.com',
    description='调试工具集：RQt 关节微调 + 一键保存。',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_relay_node = tools.joint_state_relay.joint_state_relay:main',
        ],
        'rqt_gui_py.Plugin': [
            'joint_tuner = tools.joint_tuner.joint_tuner_plugin:JointTunerPlugin',
            'joint_state_viewer = tools.joint_state_viewer.joint_state_viewer_plugin:JointStateViewerPlugin',
        ],
    },
)
