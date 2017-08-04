.PHONY: all

COMPILER = g++ -std=c++11 -O3 -g
PNAME = pose
FLAGS = -Wall

all: pose.o depthCamManager.o pointCloud.o tracker.o
	$(COMPILER) pose.o depthCamManager.o pointCloud.o tracker.o $(FLAGS) `pkg-config --cflags --libs opencv` -lrealsense -lGL -lGLU -lsfml-graphics -lsfml-window -lsfml-system -o $(PNAME)

pose.o: pose.cpp
	$(COMPILER) -c pose.cpp

depthCamManager.o: depthCamManager.cpp depthCamManager.h pointCloud.h
	$(COMPILER) -c depthCamManager.cpp

pointCloud.o: pointCloud.cpp pointCloud.h
	$(COMPILER) -c pointCloud.cpp

tracker.o: tracker.cpp tracker.h pointCloud.h
	$(COMPILER) -c tracker.cpp

.PHONY: clean
clean:
	rm -f *.o $(PNAME)