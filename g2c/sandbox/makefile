
OPTIMIZATION = -O0
DEBUG = -g3
ODEBUG = $(DEBUG)

G2C_ROOT_DIR = ..
G2C_SOURCE_DIR = $(G2C_ROOT_DIR)/g2c
ENVIRONMENTS_DIR = $(G2C_ROOT_DIR)/environments
OBJ_DIR = obj


ifneq ($(MAKECMDGOALS), clean)
CURRENT = $(MAKECMDGOALS)
current: $(CURRENT)
endif

include $(G2C_SOURCE_DIR)/platform.mk

LIBENVIRONMENTS = $(ENVIRONMENTS_DIR)/libenvironments.a
LIBG2C = $(G2C_SOURCE_DIR)/libg2c.a

$(LIBG2C): force_look
	cd $(G2C_SOURCE_DIR) && $(MAKE) libg2c.a

$(LIBENVIRONMENTS): force_look
	cd $(ENVIRONMENTS_DIR) && $(MAKE) libenvironments.a

force_look:
	true

$(OBJ_DIR)/usenamespace.h:
	echo "namespace g2c {} using namespace g2c;" >> $(OBJ_DIR)/usenamespace.h

obj:
	mkdir obj

$(CURRENT): $(LIBG2C) $(LIBENVIRONMENTS) $(G2C_SOURCE_DIR)/lin/lin.h obj $(OBJ_DIR)/usenamespace.h $(CURRENT).cpp
	c++ $(FLAGS) $(DEBUG) $(OPTIMIZATION) \
		-I$(G2C_SOURCE_DIR) \
		-I$(ENVIRONMENTS_DIR) \
		$(CURRENT).cpp -include $(OBJ_DIR)/usenamespace.h -o $(CURRENT) \
		$(LIBENVIRONMENTS) \
		$(LIBG2C) \
		$(GRAPHICS_LIBRARIES) \
		$(AUDIO_LIBRARIES)

clean:
	-rm -rf $(OBJ_DIR)/*.o
	-rm -rf *.dSYM

