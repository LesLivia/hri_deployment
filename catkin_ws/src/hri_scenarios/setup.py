from setuptools import setup

setup(
    version='0.1.0',
    scripts=['scripts/leftMotorPub.py', 'scripts/rightMotorPub.py'],
    packages=['hri_scenarios'],
    package_dir={'': 'src'}
)
