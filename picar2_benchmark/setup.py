from setuptools import find_packages, setup

package_name = 'picar2_benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/scenarios', ['scenarios/open_straight.yaml',
                                                  'scenarios/doorway.yaml',
                                                  'scenarios/dead_end_reverse.yaml']),
        ('share/' + package_name + '/launch',
         ['launch/benchmark_localization.launch.py']),
        ('share/' + package_name + '/configs',
         ['configs/rpp_dubins.yaml', 'configs/rpp_short_search.yaml',
          'configs/mppi_ackermann.yaml',
          'configs/inflation_original.yaml']),
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
