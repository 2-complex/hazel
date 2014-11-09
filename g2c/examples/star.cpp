
#include "app.h"
#include "graphics.h"
#include "fattening.h"
#include "transforms.h"

#include "launch.h"

using namespace std;
using namespace g2c;

class StarApp : public App
{
public:
    StarApp();

private:
    Bank* bank;

    double then;

    mutable Model model;

    void init();
    void resize(int width, int height);
    void setBank(Bank* bank);

    void step(double t);
    void draw() const;
};

StarApp::StarApp()
    : then(0.0)
{
}

void StarApp::setBank(Bank* bank)
{
    this->bank = bank;
}

void StarApp::init()
{
    model.bank = bank;
    bank->initSerializableWithPath(&model, "star.model");

    Polygon starLine;

    int n = 10;
    double radius = 10.0;
    for( int i = 0; i < 10; i++ )
    {
        double theta = 2.0 * M_PI * (i-0.5) / n;
        double factor =  (i % 2) ? 1.0 : 0.378;

        starLine.add(factor * radius * Vec2(cos(theta), sin(theta)));
    }

    Fattening fat;
    fat.fatten(starLine, 3.0);

    fat.populateModel(&model, model.shapes["starShape"]);
}

void StarApp::resize(int width, int height)
{
    (*(model.assumptions["context"]))["projection"] = orthographic(-width, width, -height, height, -1.0, 1.0);
    (*(model.assumptions["context"]))["modelView"] = scale(Vec3(40.0, 40.0, 1.0));
}

void StarApp::step(double t)
{
    double now = t;

    if( now - then < 1.0 / 30.0 && then != 0.0 )
        return;

    then = now;
}

void StarApp::draw() const
{
    model.draw();
}

int main(int argc, char** args)
{
    StarApp app;
    launch(&app);

    return 0;
}

