from setuptools import find_packages, setup

package_name = 'final_pro_udemy_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mbz',
    maintainer_email='mbz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "controller = final_pro_udemy_py.controller:main",
            'spawner = final_pro_udemy_py.spawner:main'
        ],
    },
)
