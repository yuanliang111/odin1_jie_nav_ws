import os
from glob import glob
from setuptools import setup

package_name = 'd1_bringup'

# 辅助函数：递归获取文件列表，用于 data_files
def package_files(directory, destination_base):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            # 这里的路径处理要非常小心
            src_path = os.path.join(path, filename)
            # 计算相对于包根目录的相对路径，用于构建安装目标路径
            rel_path = os.path.relpath(path, directory)
            if rel_path == '.':
                dest_path = destination_base
            else:
                dest_path = os.path.join(destination_base, rel_path)
            paths.append((dest_path, [src_path]))
    return paths

# 基础 data_files
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    (os.path.join('share', package_name, 'maps'), glob('maps/*.*')),
]

# 将 libraries 文件夹下的所有内容添加到 data_files
# 目标路径: share/d1_bringup/libraries/...
data_files.extend(package_files('libraries', os.path.join('share', package_name, 'libraries')))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='D1 Robot Dog Driver',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'd1_core = d1_bringup.d1_core:main',
            'simple_navigator = d1_bringup.simple_navigator:main',
            'teleop_joy = d1_bringup.teleop_joy:main',
        ],
    },
)