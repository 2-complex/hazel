
OPTIMIZATION = -O0
DEBUG = -g3
ODEBUG = $(DEBUG)

G2C_ROOT_DIR = ..
G2C_SOURCE_DIR = $(G2C_ROOT_DIR)/g2c
ENVIRONMENTS_DIR = $(G2C_ROOT_DIR)/environments
OBJ_DIR = obj

include $(G2C_SOURCE_DIR)/platform.mk

EXES = \
	atlas \
	modelfile \
	quad \
	serialize \
	ship \
	sound \
	sprite \
	star \
	steam \
	tetrahedron \
	text \
	triangle \
	worldfile \
	worldapptest

all: $(EXES)


LIBENVIRONMENTS = $(ENVIRONMENTS_DIR)/libenvironments.a
LIBG2C = $(G2C_SOURCE_DIR)/libg2c.a

$(LIBG2C): force_look
	cd $(G2C_SOURCE_DIR) && $(MAKE) libg2c.a

$(LIBENVIRONMENTS): force_look
	cd $(ENVIRONMENTS_DIR) && $(MAKE) libenvironments.a

force_look:
	true

define build_with_source
	c++ $(FLAGS) $(DEBUG) $(OPTIMIZATION) $(GLFLAGS) \
		-I$(G2C_SOURCE_DIR) -I$(ENVIRONMENTS_DIR) \
		$(1) -o $(2) \
		$(LIBENVIRONMENTS) \
		$(LIBG2C) \
		$(GRAPHICS_LIBRARIES) $(AUDIO_LIBRARIES)
endef

DEPS := $(LIBG2C) $(LIBENVIRONMENTS) $(G2C_SOURCE_DIR)/lin/lin.h

atlas: $(DEPS) atlas.cpp
	$(call build_with_source, atlas.cpp, atlas)

modelfile: $(DEPS) modelfile.cpp
	$(call build_with_source, modelfile.cpp, modelfile)

quad: $(DEPS) quad.cpp
	$(call build_with_source, quad.cpp, quad)

serialize: $(DEPS) serialize.cpp
	$(call build_with_source, serialize.cpp, serialize)

ship: $(DEPS) launch.o ship.cpp
	$(call build_with_source, launch.o ship.cpp, ship)

sound: $(DEPS) sound.cpp
	$(call build_with_source, sound.cpp, sound)

sprite: $(DEPS) launch.o sprite.cpp
	$(call build_with_source, launch.o sprite.cpp, sprite)

star: $(DEPS) launch.o star.cpp
	$(call build_with_source, launch.o star.cpp, star)

tetrahedron: $(DEPS) tetrahedron.cpp
	$(call build_with_source, tetrahedron.cpp, tetrahedron)

steam: $(DEPS) launch.o steam.cpp
	$(call build_with_source, launch.o steam.cpp, steam)

text: $(DEPS) text.cpp
	$(call build_with_source, text.cpp, text)

triangle: $(DEPS) launch.o triangle.cpp
	$(call build_with_source, launch.o triangle.cpp, triangle)

worldfile: $(DEPS) worldfile.cpp
	$(call build_with_source, worldfile.cpp, worldfile)

launch.o: $(DEPS) launch.h launch.cpp
	c++  $(FLAGS) $(DEBUG) $(OPTIMIZATION) $(GLFLAGS) \
		-I$(G2C_SOURCE_DIR) -I$(ENVIRONMENTS_DIR) \
		launch.cpp -c

worldapptest: $(DEPS) launch.o worldapptest.cpp worldapp.cpp
	$(call build_with_source, launch.o worldapptest.cpp worldapp.cpp, worldapptest)


clean:
	cd $(G2C_SOURCE_DIR) && $(MAKE) clean
	cd $(ENVIRONMENTS_DIR) && $(MAKE) clean
	-rm -f $(EXES)
	-rm -f $(OBJ_DIR)/*.o
	-rm -rf *.dSYM

