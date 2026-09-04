from glob import glob

from setuptools import find_packages, setup

package_name = 'picar2_benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # globbed, not listed: a new scenario or config that silently fails to
        # install shows up as a confusing FileNotFoundError at run time
        ('share/' + package_name + '/scenarios', glob('scenarios/*.yaml')),
        ('share/' + package_name + '/launch',
         ['launch/benchmark_localization.launch.py']),
        ('share/' + package_name + '/configs', glob('configs/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Valentin',
    maintainer_email='valentin.shevchenko@gmail.com',
    description='Reproducible Nav2 navigation benchmark for the PICAR-2 Ackermann robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bench-generate = picar2_benchmark.cli:generate_main',
            'gt_localizer = picar2_benchmark.gt_localizer:main',
            'bench-run = picar2_benchmark.runner:main',
            'bench-report = picar2_benchmark.report:main',
        ],
    },
)
