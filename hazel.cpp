
#include "transforms.h"

#include "hazel.h"

#include "sys/time.h"
#include "stdlib.h"


void HazelPrefs::rewrite() const
{
	if( !bank )
	{
		printf("Prefs rewrite with no bank.\n");
		exit(0);
	}
	bank->writePersistentSerializableWithKey(this, key.c_str());
}

Node* HazelWorld::getNode(const string& name)
{
	return World::getNode(name);
}

void HazelWorld::initWorld()
{

}


Hazel::Hazel()
{

}

Hazel::~Hazel()
{

}

void Hazel::setBank(Bank* inBank)
{
	world.bank = inBank;
}

void Hazel::setPlayer(Player* inPlayer)
{
	world.initSound(inPlayer);
}

void Hazel::init()
{
    renderer = new RendererGL2;
    Sprite::renderer = renderer;
    renderer->init();

    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_ALPHA);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    world.initWithPath("hazel.world");
    world.initWorld();
}

void Hazel::destroy()
{
	delete renderer;
}

bool Hazel::mouseDown(const Vec2& C)
{
	return world.mouseDown(C);
}

void Hazel::mouseDragged(const Vec2& C)
{	
	world.mouseDragged(C);
}

void Hazel::mouseUp(const Vec2& C)
{
	world.mouseUp(C);
}

bool Hazel::touchDown(unsigned int index, const Vec2& C)
{
	return false;
}

void Hazel::touchDragged(unsigned int index, const Vec2& C)
{
	
}

void Hazel::touchUp(unsigned int index, const Vec2& C)
{
	
}

void Hazel::draw() const
{
	glClearColor(0.1, 0.05, 0.04, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	world.draw();
}

void Hazel::keyboard(unsigned char inkey)
{

}

void Hazel::reshape(int width, int height)
{
	renderer->projection = orthographic(0, width, 0, height, -1, 1);
}

void Hazel::step(double t)
{
	
}





