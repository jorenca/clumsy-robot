from setuptools import setup
import os

package_name = 'clumsybot_init'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            val = ('share/' + package_name + '/' + path,  [path + '/' + filename])
            paths.append(val)
    return paths


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/rviz2.launch.py']),
    ] + package_files('worlds'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='georgi',
    maintainer_email='mailnamefrustration@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_sim = clumsybot_init.run_sim:main'
        ],
    },
)
