from setuptools import find_packages, setup
package_name='percepcion_item2'
setup(
    name=package_name, version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='alumno', maintainer_email='alumno@local', description='Percepcion por color P2', license='MIT',
    entry_points={'console_scripts':['percepcion_camara = percepcion_item2.percepcion_camara:main']},
)
