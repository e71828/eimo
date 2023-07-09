from setuptools import setup

package_name = 'green_bean'

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
    maintainer='e71828',
    maintainer_email='e71828@protonmail.ch',
    description='TODO: Package description',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'param_node = green_bean.parameters_node:main',
            'param_extern = green_bean.parameters_extern:main',
        ],
    },
)
