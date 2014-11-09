#include "spriteenvironment.h"

class MyEnvironment : public SpriteEnvironment
{
	void draw() const;
	void init();
	
	Bitmap bitmap;
	Sprite ladybug;
	Actor cynthia;
};

void MyEnvironment::init()
{
	bank.initBitmapWithPath(&bitmap, "ladybug.png");
	
	ladybug.initWithBitmap(bitmap);
	
	cynthia.sprite = &ladybug;
}

void MyEnvironment::draw() const
{
	cynthia.draw();
}


int main()
{
	MyEnvironment e;
	e.mainLoop();
	
	return 0;
}

