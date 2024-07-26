clean:
	rm -f *.csv GimbalTest.zip

zip:
	zip GimbalTest.zip cgs.py ground_station/ground_station.ino Makefile plane/plane.ino
