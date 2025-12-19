from setuptools import find_packages, setup

package_name = 'yudha_pkg'

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
    maintainer='bluefin',
    maintainer_email='iniakureal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "publisher_yudha = yudha_pkg.publisher:main",
            "subscriber_yudha = yudha_pkg.subscriber:main",
        ],
    },
)
