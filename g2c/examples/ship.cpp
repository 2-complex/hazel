
#include "app.h"
#include "sprites.h"
#include "launch.h"
#include "transforms.h"

using namespace std;
using namespace g2c;

class ShipApp : public App
{
public:
    ShipApp();

private:
    Sprite ship;
    Actor enterprise;
    Bank* bank;
    RendererGL2 renderer;

    set<unsigned char> keys;

    Vec2 velocity;
    Vec2 position;
    double theta;

    double then;

    void init();
    void resize(int width, int height);
    void setBank(Bank* bank);

    void step(double t);
    void draw() const;

    void keyDown(unsigned char c);
    void keyUp(unsigned char c);
};

ShipApp::ShipApp()
    : theta(0.0)
    , then(0.0)
{
}

void ShipApp::keyDown(unsigned char c)
{
    keys.insert(c);
}

void ShipApp::keyUp(unsigned char c)
{
    keys.erase(c);
}

void ShipApp::setBank(Bank* bank)
{
    this->bank = bank;
}

void ShipApp::init()
{
    bank->initTextureWithPath(&ship, "ship.png");
    ship.numberOfRows = 2;
    ship.numberOfColumns = 32;
    ship.center = true;
    ship.flipRows = true;

    enterprise.sprite = &ship;
    enterprise.position.set(100, 100);

    renderer.init();
    Mesh::renderer = &renderer;
}

void ShipApp::resize(int width, int height)
{
    renderer.projection = orthographic(0,width,0,height,-1,1);
}

void ShipApp::step(double t)
{
    double now = t;

    if( now - then < 1.0 / 30.0 && then != 0.0 )
        return;

    if( keys.find('a') != keys.end() )
        theta += M_PI / 16.0;

    if( keys.find('d') != keys.end() )
        theta -= M_PI / 16.0;

    bool thrust = false;
    if( keys.find('s') != keys.end() ||
        keys.find('w') != keys.end() )
    {
        thrust = true;
        velocity += Vec2(cos(theta), sin(theta));
    }

    enterprise.position += velocity;
    enterprise.frame = (((int)(32.0 * (theta / (2.0 * M_PI)) + 0.5)) % 32 + 32) % 32;

    if( thrust )
        enterprise.frame += 32;

    then = now;
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

