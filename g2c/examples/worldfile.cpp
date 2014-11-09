
#include "spriteenvironment.h"
#include "sprites.h"

#include <stdio.h>

class MySpriteEnvironment : public SpriteEnvironment
{
public:
	char* filename;

private:
	void init();
};

void MySpriteEnvironment::init()
{
	bank.initSerializableWithPath(&world, filename);
	printf( "%s\n", world.serialize().c_str() );
}

int main(int argc, char** args)
{
	MySpriteEnvironment e;

	e.filename = (char*)"bug.world";
	if( argc > 1 )
		e.filename = args[1];
	
	e.mainLoop();
	
	return 0;
}

