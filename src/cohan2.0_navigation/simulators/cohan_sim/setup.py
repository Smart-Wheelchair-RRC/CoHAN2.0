from setuptools import setup, find_packages

package_name = 'cohan_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Phani Teja Singamaneni',
    maintainer_email='ptsingaman@laas.fr',
    description='cohan_sim for simple 2d simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_sim = cohan_sim.run_sim:main',
        ],
    },
)
