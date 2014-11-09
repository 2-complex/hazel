
#include "app.h"
#include "sprites.h"
#include "transforms.h"

#include "launch.h"

using namespace g2c;

class ShipApp : public App
{
    Sprite ship;
    Actor enterprise;
    Bank* bank;
    RendererGL2 renderer;

    void init();
    void resize(int width, int height);
    void draw() const;

    void setBank(Bank* bank);
};

void ShipApp::setBank(Bank* bank)
{
    this->bank = bank;
}

void ShipApp::resize(int width, int height)
{
    renderer.projection = orthographic(0,width,0,height,-1,1);
}

void ShipApp::init()
{
    Mesh::renderer = &renderer;
    renderer.init();

    bank->initTextureWithPath(&ship, "ship.png");
    ship.numberOfRows = 2;
    ship.numberOfColumns = 32;

    enterprise.sprite = &ship;
    enterprise.x = 100;
    enterprise.y = 200;
    enterprise.frame = 7;
}

void ShipApp::draw() const
{
    enterprise.draw();
}

int main(int argc, char** args)
{
    ShipApp app;
    launch(&app);
    return 0;
}

