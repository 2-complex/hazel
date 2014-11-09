
#include "worldapp.h"
#include "sprites.h"
#include "transforms.h"
#include "rendererg.h"

using namespace g2c;

WorldApp::WorldApp()
{
}

WorldApp::~WorldApp()
{
}

void WorldApp::setBank(Bank* inBank)
{
    this->bank = inBank;
}

void WorldApp::resize(int width, int height)
{
    renderer->projection = orthographic(0, width, 0, height, -1, 1);
}

void WorldApp::init()
{
    world = new World;
    renderer =  new RendererG;
    renderer->init();

    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_ALPHA);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    Mesh::renderer = renderer;

    world->bank = bank;
    bank->initSerializableWithPath(world, "bug.world");
    g2clog( "%s\n", world->serialize().c_str() );
}

void WorldApp::draw() const
{
    world->draw();
}


void WorldApp::destroy()
{
    delete world;
    delete renderer;
}

