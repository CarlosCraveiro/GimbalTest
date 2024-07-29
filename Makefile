run:
	python control_groundstation.py --port /dev/ACM0 --baud 57600

clean:
	rm -f *.csv GimbalTest.zip
	rm -rf __pycache__/

zip:
	zip GimbalTest.zip cgs.py ground_station/ground_station.ino Makefile plane/plane.ino
