
#include "panenvironment.h"
#include "graphics.h"

#if __APPLE_CC__
#include "macbank.h"
#else
#include "unixbank.h"
#endif

#include <stdio.h>

using namespace g2c;

class MyPanEnvironment : public PanEnvironment {
public:
	char* filename;

#if __APPLE_CC__
	MacFileSystemBank bank;
#else
	UnixBank bank;
#endif

	mutable Model model;

private:
	void init();
	void draw() const;
};

void MyPanEnvironment::init()
{
	initOpenGL();
	
	model.bank = &bank;
	
	bank.initSerializableWithPath(&model, filename);
	printf( "%s\n", model.serialize().c_str() );
}

void MyPanEnvironment::draw() const
{
	Assumption* context = model.assumptions["context"];
	(*context)["modelView"] = getModelView();
	(*context)["projection"] = getProjection();

	model.draw();
}

int main(int argc, char** args)
{
	MyPanEnvironment e;
	
	e.filename = (char*)"box.model";
	if( argc > 1 )
		e.filename = args[1];
	
	e.mainLoop();
	
	return 0;
}

