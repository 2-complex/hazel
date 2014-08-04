
#ifndef _GAME_
#define _GAME_

#include "lib2d/phys2d.h"
#include "sprites.h"
#include "graphics.h"

#include <vector>

using namespace std;
using namespace g2c;


enum CollisionCode
{
    kObject = 0,
    kCraft = 1,
    kUpdraft = 2,
    kTree = 3
};

class Object
{
public:
    Object();
    virtual ~Object();

    CollisionCode collisionCode;
    Actor* actor;

    lib2d::Polygon p;
    lib2d::Body* body;

    virtual void initBody();

    virtual void step();
    virtual void draw() const;
    virtual void drawTranslucent() const;
};

class Craft
    : public Object
{
public:
    explicit Craft(Sampler*);
    virtual ~Craft();

    virtual void initBody();

    virtual void step();
};

class Updraft
    : public Object
{
public:
    explicit Updraft(Sampler*);
    virtual ~Updraft();

    virtual void step();
    virtual void draw() const;
    virtual void drawTranslucent() const;
};

class Land
    : public Object
{
public:
    explicit Land();
    virtual ~Land();

    virtual void initBody();
};

class Tree
    : public Object
{
public:
    explicit Tree(Sampler*);
    virtual ~Tree();

    virtual void initBody();
};

class GameLayer;

class Game
{
friend class GameLayer;

public:
    Game();
    virtual ~Game();

private:
    vector<Object*> objects;
    Craft* primaryCraft;

    Model landModel;

    lib2d::Universe universe;

    Layer* steamLayer;
    GameLayer* gameLayer;

    double lastTime;
    set<unsigned char> keySet;

public:
    void init(World* world);
    void destroy();
    void step(double t);

    virtual void keyDown(unsigned char inkey);
    virtual void keyUp(unsigned char inkey);

    void drawPhysics() const;

private:
    void addObject(Object* o);
    void handleCollisions();
};

class GameLayer : public Layer
{
public:
    explicit GameLayer(Game* game);
    virtual ~GameLayer();

    Game* game;

    virtual void drawInTree(
        const Mat4& worldMatrix,
        const Color& worldColor) const;
};

#endif

