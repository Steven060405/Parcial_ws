from setuptools import find_packages, setup
package_name='transcriptor_voz'
setup(
    name=package_name, version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml'])],
    install_requires=['setuptools','requests'], zip_safe=True,
    maintainer='alumno', maintainer_email='alumno@local', description='Transcriptor de voz P3', license='MIT',
    entry_points={'console_scripts':['transcriptor_voz = transcriptor_voz.transcriptor_voz:main']},
)
