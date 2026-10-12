from setuptools import find_packages, setup

package_name = 'parcial_interprete'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='alumno',
    maintainer_email='alumno01@esan.edu.pe',
    description='Interprete de ordenes del parcial JetCobot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'interprete_ordenes = parcial_interprete.interprete_ordenes:main',
        ],
    },
)
