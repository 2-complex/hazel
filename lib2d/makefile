CC = c++
OPTIMIZATION = -O3
DEBUGGER = -g
PROFILER = 
INCLUDES = -I../../g2c/g2c

FILES = math2d.o phys2d.o


UNAME := $(shell uname)

ifeq ($(UNAME), Darwin)
	GL_LIBRARIES = -framework OpenGL -framework Cocoa  -framework glut
endif
ifeq ($(UNAME), Linux)
	GL_LIBRARIES = -lGL -lGLU -lglut
endif

math2d.o: math2d.cpp
	$(CC) -c $(OPTIMIZATION) $(INCLUDES) -DGL_ON math2d.cpp $(PROFILER) $(DEBUGGER)

phys2d.o: phys2d.cpp
	$(CC) -c $(OPTIMIZATION) $(INCLUDES) -DGL_ON phys2d.cpp $(PROFILER) $(DEBUGGER)

prof_test: math2d.o phys2d.o prof_test.cpp
	$(CC) $(OPTIMIZATION) $(INCLUDES) $(GL_LIBRARIES) $(FILES) prof_test.cpp -o prof_test $(PROFILER) $(DEBUGGER)

test:  math2d.o phys2d.o test.cpp
	$(CC) $(OPTIMIZATION) $(INCLUDES) $(FILES) test.cpp -o test $(PROFILER) $(DEBUGGER)

gl_test: math2d.o phys2d.o gl_test.cpp
	$(CC) $(OPTIMIZATION) $(FRAMEWORKS) $(FILES) gl_test.cpp -o gl_test

acid_test: math2d.o phys2d.o acid_test.cpp
	$(CC) $(OPTIMIZATION) $(GL_LIBRARIES) $(INCLUDES) -DGL_ON $(FILES) acid_test.cpp -o acid_test $(PROFILER) $(DEBUGGER)

three_test: math2d.o phys2d.o three_test.cpp
	$(CC) $(OPTIMIZATION) $(GL_LIBRARIES) $(FILES) three_test.cpp -o three_test $(PROFILER) $(DEBUGGER)

simple_test: math2d.o phys2d.o simple_test.cpp
	$(CC) $(OPTIMIZATION) $(GL_LIBRARIES) $(FILES) simple_test.cpp -o simple_test $(PROFILER) $(DEBUGGER)

impulse: math2d.o phys2d.o impulse.cpp
	$(CC) $(OPTIMIZATION) $(GL_LIBRARIES) $(FILES) impulse.cpp -o impulse

space_tree: math2d.o phys2d.o space_tree.cpp
	$(CC) $(OPTIMIZATION) $(GL_LIBRARIES) $(FILES) space_tree.cpp -o space_tree

piecetester: math2d.o phys2d.o piecetester.cpp
	$(CC) $(OPTIMIZATION) $(GL_LIBRARIES) $(FILES) piecetester.cpp -o piecetester

mover: math2d.o phys2d.o mover.cpp
	$(CC) $(OPTIMIZATION) $(GL_LIBRARIES) $(FILES) mover.cpp -o mover

perimeter_paths: perimeter_paths.cpp phys2d.o math2d.o
	$(CC) $(OPTIMIZATION) $(GL_LIBRARIES) $(FILES) perimeter_paths.cpp -o perimeter_paths


clean:
	\rm *.o


