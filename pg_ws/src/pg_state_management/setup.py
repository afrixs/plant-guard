from setuptools import find_packages, setup

package_name = 'pg_state_management'

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
    maintainer='Matej Vargovcik',
    maintainer_email='matej.vargovcik@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_server = pg_state_management.state_server:main',
            'state_client = pg_state_management.state_client:main',
        ],
    },
)
