.PHONY: all

COMPILER = g++ -std=c++11
SOURCE = depthCamManager
PNAME = pose
FLAGS = -Wall

all: depthCamManager.o
	$(COMPILER) $(SOURCE).o $(FLAGS) `pkg-config --cflags --libs opencv` -lrealsense -o $(PNAME)

depthCamManager.o: depthCamManager.cpp depthCamManager.h
	$(COMPILER) -c $(SOURCE).cpp

.PHONY: clean
clean:
	rm -f *.o $(PNAME)
