from setuptools import setup
import glob
import os

package_name = 'drone_control'

def get_data_files():
    """Manually specify data_files list."""
    target = "share/" + package_name
    files = [
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ]

    files.extend(recurse_data_files("launch", target))

    return files

def recurse_data_files(source_dir: str, target_dir: str):
    """Traverse source_dir and append each file found to a new entry in the files list.

    Intended for use with data_file since it expects a file as a source.

    Example output: [("share/mypackage/worlds", ["worlds/moon.world"])]
    """
    data_files = []

    for file in filter(
        lambda f: os.path.isfile(f),
        glob.iglob("{}/**/*".format(source_dir), recursive=True),
    ):
        target = "{}/{}".format(target_dir, os.path.dirname(file))
        data_files.append((target, [file]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cnchano',
    maintainer_email='cnchano@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'state_publisher = {package_name}.state_publisher:main',
            f'gps_subscriber = {package_name}.gps_subscriber:main',
            f'imu_subscriber = {package_name}.imu_subscriber:main',
            f'clock_subscriber = {package_name}.clock_subscriber:main',
            #f'front_pt_cloud_subscriber = {package_name}.front_pt_cloud_subscriber:main',
            #f'ptcloud_func_tools = {package_name}.ptcloud_func_tools:main',
            f'drone_controller_conductor = {package_name}.drone_controller_conductor:main',
            f'pos_controller = {package_name}.pos_controller:main',
            f'behavioral_controller = {package_name}.behavioral_controller:main',
            #f'mpc_controller = {package_name}.mpc_controller:main',
            f'velocity_calc_service = {package_name}.velocity_calc_service:main',
            f'attitude_controller = {package_name}.attitude_controller:main',
            f'drone_com_out = {package_name}.drone_com_out:main',
            f'drone_com_in = {package_name}.drone_com_in:main',
            f'cmd_fwd_publisher = {package_name}.cmd_fwd_publisher:main',
            
        ],
    },
)
