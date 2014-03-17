CC = c++
OPTIMIZATION = -force_cpusubtype_ALL -O3
DEBUGGER = -g
PROFILER = 
INCLUDES = -I../g2c/g2c

FILES = math2d.o phys2d.o
FRAMEWORKS = -framework OpenGL -framework GLUT -framework Cocoa

math2d.o: math2d.cpp
	$(CC) -c $(OPTIMIZATION) $(INCLUDES) math2d.cpp $(PROFILER) $(DEBUGGER)

phys2d.o: phys2d.cpp
	$(CC) -c $(OPTIMIZATION) $(INCLUDES) phys2d.cpp $(PROFILER) $(DEBUGGER)

prof_test: math2d.o phys2d.o prof_test.cpp
	$(CC) $(OPTIMIZATION) $(INCLUDES) $(FRAMEWORKS) $(FILES) prof_test.cpp -o prof_test $(PROFILER) $(DEBUGGER)

test:  math2d.o phys2d.o test.cpp
	$(CC) $(OPTIMIZATION) $(INCLUDES) $(FRAMEWORKS) $(FILES) test.cpp -o test $(PROFILER) $(DEBUGGER)

gl_test: math2d.o phys2d.o gl_test.cpp
	$(CC) $(OPTIMIZATION) $(FRAMEWORKS) $(FILES) gl_test.cpp -o gl_test

acid_test: math2d.o phys2d.o acid_test.cpp
	$(CC) $(OPTIMIZATION) $(FRAMEWORKS) $(FILES) acid_test.cpp -o acid_test $(PROFILER) $(DEBUGGER)

three_test: math2d.o phys2d.o three_test.cpp
	$(CC) $(OPTIMIZATION) $(FRAMEWORKS) $(FILES) three_test.cpp -o three_test $(PROFILER) $(DEBUGGER)

simple_test: math2d.o phys2d.o simple_test.cpp
	$(CC) $(OPTIMIZATION) $(FRAMEWORKS) $(FILES) simple_test.cpp -o simple_test $(PROFILER) $(DEBUGGER)

impulse: math2d.o phys2d.o impulse.cpp
	$(CC) $(OPTIMIZATION) $(FRAMEWORKS) $(FILES) impulse.cpp -o impulse

space_tree: math2d.o phys2d.o space_tree.cpp
	$(CC) $(OPTIMIZATION) $(FRAMEWORKS) $(FILES) space_tree.cpp -o space_tree

piecetester: math2d.o phys2d.o piecetester.cpp
	$(CC) $(OPTIMIZATION) $(FRAMEWORKS) $(FILES) piecetester.cpp -o piecetester

mover: math2d.o phys2d.o mover.cpp
	$(CC) $(OPTIMIZATION) $(FRAMEWORKS) $(FILES) mover.cpp -o mover

perimeter_paths: perimeter_paths.cpp phys2d.o math2d.o
	$(CC) $(OPTIMIZATION) $(FRAMEWORKS) $(FILES) perimeter_paths.cpp -o perimeter_paths




clean:
	\rm *.o

