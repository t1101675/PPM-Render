ifeq ("$(CC)", "icpc")
LOMP = -qopenmp
CC = icpc
OPT = -xHost -O3 -ip -ipo
else
LOMP = -fopenmp
CC = g++
OPT= 
endif
CFLAGS = -Wall -std=c++11 $(OPT) $(LOMP)
LDFLAGS = -Wall -lopencv_imgcodecs -lopencv_calib3d -lopencv_imgproc -lopencv_core -lopencv_ml -lopencv_features2d -lopencv_objdetect -lopencv_flann -lopencv_video -lopencv_highgui
LDLIBS = $(LDFLAGS) -ljson $(LOMP)

TARGETS = main 
OBJECTS = main.o scene.o render.o object.o plane.o ball.o light.o vec3.o utils.o bezier.o b_obj.o sub_ball.o kdtree.o mesh_obj.o
HEADERS = object.h scene.h render.h utils.h bezier.h
HPPS = plane.hpp ball.hpp light.hpp vec3.hpp b_obj.hpp sub_ball.hpp kdtree.hpp mesh_obj.hpp

.PHONY: default
default: all

.PHONY: all
all: $(TARGETS)

main: main.o scene.o render.o object.o utils.o bezier.o
	$(CC) -o $@ $^ $(LDLIBS)

%.o: %.cpp $(HEADERS) $(HPPS)
	$(CC) -c $(CFLAGS) $< -o $@

%.o: %.hpp $(HEADERS)
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	rm *.o main

.PHONY: clean
clean:
	rm -rf $(TARGETS) $(OBJECTS)
