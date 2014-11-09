
UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
	GRAPHICS_LIBRARIES = \
		-framework Cocoa \
		-framework QuartzCore \
		-framework GLUT \
		-framework OpenGL
	AUDIO_LIBRARIES = \
		-framework AudioToolbox \
		-framework Foundation \
		-framework OpenAL
endif
ifeq ($(UNAME), Linux)
	GRAPHICS_LIBRARIES = -lGL -lGLU -lGLEW -lglut -lpng
	AUDIO_LIBRARIES = -lopenal
endif

