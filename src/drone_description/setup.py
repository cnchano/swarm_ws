from setuptools import setup
import glob
import os

package_name = 'drone_description'

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
    files.extend(recurse_data_files("meshes", target))
    files.extend(recurse_data_files("urdf", target))
    # files.extend(recurse_data_files("drone_description", target))
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
            f'spawn_drone_exe = {package_name}.spawn_drone_exe:main',
            
        ],
    },
) 

