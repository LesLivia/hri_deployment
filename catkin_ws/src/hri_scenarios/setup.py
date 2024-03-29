from setuptools import setup

setup(
    version='0.1.0',
    scripts=['scripts/leftMotorPub.py', 'scripts/rightMotorPub.py', 'scripts/allMotorPub.py',
		'scripts/humSensorsSub.py', 'scripts/robSensorsSub.py', 'scripts/robBatterySub.py', 
		'scripts/humFtgSub.py', 'scripts/robTrajPub.py', 'scripts/robStatusPub.py', 
		'scripts/humServiceSub.py', 'scripts/missionStatusPub.py', 'scripts/ttb3cmdvel.py',
		'scripts/ttb3cmdnav.py', 'scripts/ttb3subchg.py', 'scripts/ttb3subpos.py'],
    packages=['hri_scenarios'],
    package_dir={'': 'src'}
)
