import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'd1_description'

# 这个函数用于递归查找文件夹内的所有文件，保持目录结构
def package_files(directory, destination_base):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            src_path = os.path.join(path, filename)
            rel_path = os.path.relpath(path, directory)
            if rel_path == '.':
                dest_path = destination_base
            else:
                dest_path = os.path.join(destination_base, rel_path)
            paths.append((dest_path, [src_path]))
    return paths

# --- 构建 data_files 列表 ---
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
]

# --- 调用辅助函数将 meshes 加入列表 ---
# 这会将 src/d1_description/meshes 下的所有内容安装到 install/share/d1_description/meshes
data_files.extend(package_files('meshes', os.path.join('share', package_name, 'meshes')))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
