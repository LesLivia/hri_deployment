from setuptools import setup

setup(
    version='0.1.0',
    scripts=['scripts/leftMotorPub.py', 'scripts/rightMotorPub.py', 'scripts/allMotorPub.py',
		'scripts/humSensorsSub.py', 'scripts/robSensorsSub.py'],
    packages=['hri_scenarios'],
    package_dir={'': 'src'}
)
