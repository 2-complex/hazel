
#include "spriteenvironment.h"

class TextEnvironment : public SpriteEnvironment {
	Font font;
	Text text;
	
	void init();
	void draw() const;
};

void TextEnvironment::init()
{
	Bitmap bitmap;
	bank.initBitmapWithPath(&bitmap, "helv.png");
	font.initWithBitmap(bitmap);
	font.numberOfRows = 6;
	font.numberOfColumns = 16;
	
	font.getWidthsFromBitmap(bitmap);
	
	text.font = &font;
	text.s = "My Message";
	text.x = 100;
	text.y = 100;
}

void TextEnvironment::draw() const
{
	text.draw();
}

int main()
{
	TextEnvironment e;
	e.mainLoop();
	
	return 0;
}
