
#include "bank.h"
#include "sprites.h"
#include "app.h"

using namespace g2c;

class WorldApp : public App
{
public:
    WorldApp();
    virtual ~WorldApp();

    virtual void setBank(Bank* inBank);

    virtual void init();
    virtual void resize(int width, int height);
    virtual void draw() const;
    virtual void destroy();

private:
    Bank* bank;
    World* world;
    Renderer* renderer;
};


