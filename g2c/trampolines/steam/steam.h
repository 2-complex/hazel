
#ifndef _STEAM_
#define _STEAM_

#include "graphics.h"
#include "app.h"

using namespace std;
using namespace g2c;

struct Particle
{
    Particle(const Vec3& inCenter, double inRadius)
    {
        center = inCenter;
        radius = inRadius;

        deltaCenter = Vec3(
            (randInt(110) - 50) / 1500.0,
            (randInt(110) - 50) / 1500.0,
            0.3);
        deltaRadius = 0.1;
    }

    Vec3 center;
    double radius;

    Vec3 deltaCenter;
    double deltaRadius;
};

class SteamApp : public App
{
    Buffer buffer;
    IndexBuffer indexBuffer;
    Geometry geometry;
    Effect effect;
    Assumption material;
    mutable Assumption particle;
    mutable Assumption camera;
    Shape shape;

    double last_t;
    vector<Particle> particles;

    double cameraRadius;
    Mat4 projection;
    Mat4 modelView;
    Bank* bank;

public:
    void init();
    void reshape(int width, int height);
    void step(double t);
    void draw() const;

    void setBank(Bank* bank);
};

#endif

