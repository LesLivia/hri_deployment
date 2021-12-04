for((i=1;i<=$2;i++))
do
	# python3 randomize_mission.py
	gnome-terminal -e "python3 main.py"
	# python3 main.py > outfile.txt &
	sleep 15
	python3 hum_main.py
	NEW_DIR=SIM_"$(date +"%Y-%m-%d-%H-%M-%S")"
	mkdir /media/psf/logs/sim_logs/faoc_exp/$1/$NEW_DIR
	cp outfile.txt /media/psf/logs/sim_logs/faoc_exp/$1/$NEW_DIR/
	cp ../scene_logs/humanFatigue.log /media/psf/logs/sim_logs/faoc_exp/$1/$NEW_DIR/
	cp ../scene_logs/humanPosition.log /media/psf/logs/sim_logs/faoc_exp/$1/$NEW_DIR/
	cp ../scene_logs/humansServed.log /media/psf/logs/sim_logs/faoc_exp/$1/$NEW_DIR/
	cp ../scene_logs/robotPosition.log /media/psf/logs/sim_logs/faoc_exp/$1/$NEW_DIR/
	cp ../scene_logs/robotBattery.log /media/psf/logs/sim_logs/faoc_exp/$1/$NEW_DIR/
	cp ../scene_logs/environmentData.log /media/psf/logs/sim_logs/faoc_exp/$1/$NEW_DIR/
	sleep 15
	python3 sim_analysis.py $1
	rosrun hri_scenarios ttb3cmdnav.py 1.00 0.28
done
