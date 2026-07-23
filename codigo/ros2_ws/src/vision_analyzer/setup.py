from setuptools import find_packages, setup

package_name = 'vision_analyzer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboticaandaja',
    maintainer_email='roboticaandaja@todo.todo',
    description='ROS2 node that captures images from usb_cam and analyzes them using Google Gemini API',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'image_analyzer = vision_analyzer.image_analyzer:main',
        ],
    },
)
