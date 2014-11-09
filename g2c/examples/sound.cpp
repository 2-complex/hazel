
#include "spriteenvironment.h"

class SoundEnvironment : public SpriteEnvironment {
	Sound sound;
	Source source;
	
	void init();
	bool mouseDown(const Vec2& c);
};

void SoundEnvironment::init()
{
	bank.initSoundWithPath(&sound, "harmonica.wav");
	sound.source = &source;
}

bool SoundEnvironment::mouseDown(const Vec2& c)
{
	sound.play();
	return true;
}

int main()
{
	SoundEnvironment e;
	e.mainLoop();
	
	return 0;
}

