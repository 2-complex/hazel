
CURRENT = apptest
ARGUMENTS = 

OPTIMIZATION = -O0
DEBUG = -g3
ODEBUG = $(DEBUG)

ENVIRONMENTS_DIR = g2c/environments
LIBG2C_DIR = g2c/g2c

LIBG2C = $(LIBG2C_DIR)/libg2c.a
LIBENVIRONMENTS = $(ENVIRONMENTS_DIR)/libenvironments.a

include $(LIBG2C_DIR)/platform.mk

HAZEL = hazel.o worldbase.o
LIB2D = math2d.o phys2d.o

current: $(CURRENT)

run: $(CURRENT)
	./$(CURRENT) $(ARGUMENTS)

$(LIBG2C): force_look
	$(MAKE) -C $(LIBG2C_DIR) libg2c.a

$(LIBENVIRONMENTS): force_look
	$(MAKE) -C $(ENVIRONMENTS_DIR) libenvironments.a

apptest: $(LIBG2C) $(LIBENVIRONMENTS) $(LIB2D) $(HAZEL) apptest.cpp
	c++ $(FLAGS) $(DEBUG) $(OPTIMIZATION) $(GLFLAGS) $(AUDIOFLAGS) \
		-I$(ENVIRONMENTS_DIR) \
		-I$(LIBG2C_DIR) \
		apptest.cpp -o apptest \
		$(LIBENVIRONMENTS)\
		$(LIBG2C) \
		$(HAZEL) \
		$(LIB2D) \
		$(GRAPHICS_LIBRARIES) \
		$(AUDIO_LIBRARIES)


worldbase.cpp: hazel.world basegen.py
	python basegen.py hazel.world WorldBase

worldbase.h: hazel.world basegen.py
	python basegen.py hazel.world WorldBase

worldbase.o: worldbase.cpp worldbase.h
	c++ $(FLAGS) -c $(DEBUG) $(OPTIMIZATION) -I$(LIBG2C_DIR) worldbase.cpp

math2d.o: lib2d/math2d.cpp lib2d/math2d.h
	c++ -c $(DEBUG) -Ig2c/g2c $(OPTIMIZATION) -DGL_ON=0 lib2d/math2d.cpp

phys2d.o: lib2d/phys2d.cpp lib2d/phys2d.h lib2d/math2d.h
	c++ -c $(DEBUG) -Ig2c/g2c $(OPTIMIZATION) -DGL_ON=0 lib2d/phys2d.cpp

hazel.o: hazel.cpp hazel.h
	c++ $(FLAGS) -c $(DEBUG) $(OPTIMIZATION) -I$(LIBG2C_DIR) hazel.cpp


.PHONY: force_look

clean:
	rm -f *.o
	rm -f worldbase.cpp
	rm -f worldbase.h

clean-all:
	rm -f *.o
	rm -f worldbase.cpp
	rm -f worldbase.h
	rm -f g2c/g2c/obj/*.o

