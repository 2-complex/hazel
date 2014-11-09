
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

void HazelWorld::destroyWorld()
{
}

Hazel::Hazel()
	: lastTime(0.0)
    , drawPhysicsOn(false)
{

}

Hazel::~Hazel()
{

}

void Hazel::setBank(Bank* inBank)
{
    world.bank = inBank;
}

void Hazel::setAudioPlayer(AudioPlayer* inPlayer)
{
    world.initSound(inPlayer);
}

void Hazel::init()
{
    renderer = new RendererGL2;
    Mesh::renderer = renderer;
    renderer->init();

    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_ALPHA);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

    world.initWithPath("hazel.world");
    world.init();
    world.initWorld();

    game.init(&world);
}

void Hazel::destroy()
{
    game.destroy();

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

#if GLUT
    if( drawPhysicsOn )
    {
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glCullFace(GL_NONE);
        glEnable(GL_ALPHA);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        gluOrtho2D(0.0, windowWidth, 0.0, windowHeight);

        game.drawPhysics();

        glMatrixMode(GL_PROJECTION);
        glPopMatrix();

        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();
    }
#endif

}

void Hazel::keyboard(unsigned char inkey)
{
    if (inkey == 'p')
    {
        drawPhysicsOn = !drawPhysicsOn;
    }
}

void Hazel::keyDown(unsigned char inkey)
{
    game.keyDown(inkey);
}

void Hazel::keyUp(unsigned char inkey)
{
    game.keyUp(inkey);
}

void Hazel::resize(int width, int height)
{
    renderer->projection = orthographic(0, width, 0, height, -1, 1);
    windowWidth = width;
    windowHeight = height;
}

void Hazel::step(double t)
{
    game.step(t);
}

