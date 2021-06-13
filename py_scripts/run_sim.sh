for i in {1..1}
do
	python3 randomize_mission.py
	python3 main.py > outfile.txt &
	sleep 2
	python3 hum_main.py
	NEW_DIR=SIM_"$(date +"%Y-%m-%d-%H-%M-%S")"
	mkdir /media/psf/logs/sim_logs/$NEW_DIR
	cp ../scene_logs/humanFatigue.log /media/psf/logs/sim_logs/$NEW_DIR/
	cp ../scene_logs/humanPosition.log /media/psf/logs/sim_logs/$NEW_DIR/
	cp ../scene_logs/humansServed.log /media/psf/logs/sim_logs/$NEW_DIR/
	cp ../scene_logs/robotPosition.log /media/psf/logs/sim_logs/$NEW_DIR/
	cp ../scene_logs/robotBattery.log /media/psf/logs/sim_logs/$NEW_DIR/
	cp ../scene_logs/environmentData.log /media/psf/logs/sim_logs/$NEW_DIR/
	sleep 5
done
