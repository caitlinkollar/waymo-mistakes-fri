from setuptools import setup

package_name = 'local_stt_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'SpeechRecognition', 'pocketsphinx'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 local STT subscriber node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_stt_subscriber = local_stt_pkg.local_stt_subscriber:main',
        ],
    },
)
