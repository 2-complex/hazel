
CURRENT = apptest
ARGUMENTS = 

OPTIMIZATION = -O0
DEBUG = -g3
ODEBUG = $(DEBUG)
OSTYPE = $(shell uname -msr)

FLAGS = -DGLUT

ifeq ($(findstring Linux,$(OSTYPE)),Linux)
	GLFLAGS = -lglut -lGLU -lGL -L/usr/X11R6/lib/ -lXmu -lXi -lXext -lX11 -lXt -lpthread
endif

ifeq ($(findstring CYGWIN,$(OSTYPE)),CYGWIN)
	GLFLAGS = -L/usr/X11R6/lib -L/usr/lib/w32api -lglut32 -lglu32 -lopengl32 -lXm -lpthread
endif

ifeq ($(findstring Darwin,$(OSTYPE)),Darwin)
	GLFLAGS = -framework Cocoa -framework OpenGL -framework GLUT
	PNGFLAGS = -framework libpng
	AUDIOFLAGS = -framework AudioToolbox -framework Foundation -framework OpenAL
	GRAPHICSFLAGS = -framework Cocoa -framework QuartzCore
endif


ENVIRONMENTS_DIR = g2c/environments
ENVIRONMENTS = environment.o appenvironment.o

LIBG2CDIR = g2c/g2c
LIBG2C = $(LIBG2CDIR)/libg2c.a

HAZEL = hazel.o worldbase.o

current: $(CURRENT)

run: $(CURRENT)
	./$(CURRENT) $(ARGUMENTS)

$(LIBG2C): force_look
	$(MAKE) -C $(LIBG2CDIR) libg2c.a

apptest: $(LIBG2C) $(HAZEL) apptest.cpp $(ENVIRONMENTS)
	c++ $(FLAGS) $(DEBUG) $(OPTIMIZATION) $(GLFLAGS) $(AUDIOFLAGS) \
		$(ENVIRONMENTS) $(LIBG2C) \
		$(HAZEL) \
		-I$(ENVIRONMENTS_DIR) -I$(LIBG2CDIR) \
		apptest.cpp -o apptest


worldbase.cpp: hazel.world basegen.py
	python basegen.py hazel.world WorldBase

worldbase.h: hazel.world basegen.py
	python basegen.py hazel.world WorldBase

worldbase.o: worldbase.cpp worldbase.h
	c++ $(FLAGS) -c $(DEBUG) $(OPTIMIZATION) -I$(LIBG2CDIR) worldbase.cpp

hazel.o: hazel.cpp hazel.h
	c++ $(FLAGS) -c $(DEBUG) $(OPTIMIZATION) -I$(LIBG2CDIR) hazel.cpp

environment.o: $(ENVIRONMENTS_DIR)/environment.h $(ENVIRONMENTS_DIR)/environment.cpp
	c++ $(FLAGS) -c $(DEBUG) $(OPTIMIZATION) -I$(LIBG2CDIR) $(ENVIRONMENTS_DIR)/environment.cpp

appenvironment.o: $(ENVIRONMENTS_DIR)/appenvironment.h $(ENVIRONMENTS_DIR)/appenvironment.cpp
	c++ $(FLAGS) -c $(DEBUG) $(OPTIMIZATION) -I$(LIBG2CDIR) $(ENVIRONMENTS_DIR)/appenvironment.cpp


.PHONY: force_look

clean:
	rm -f *.o
	rm -f worldbase.cpp
	rm -f worldbase.h
	rm -f g2c/g2c/obj/*.o

