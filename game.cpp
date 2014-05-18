
#include "game.h"

using namespace lib2d;

Game::Game()
    : primaryCraft(NULL)
    , lastTime(0.0)
{
}

Game::~Game()
{
}

void Game::init(World* world)
{
    primaryCraft = new Craft(world->getSprite("hazelSprite"));
    addObject(primaryCraft);

    primaryCraft->body->position.set(100, 300);

    Layer* gameContainer = (Layer*)world->findChild("gameContainer");

    gameLayer = new GameLayer(this);
    steamLayer = new Layer;

    gameContainer->add(gameLayer);
    gameContainer->add(steamLayer);

    for( int i = 0; i < 3; i++ )
    {
        Updraft* up = new Updraft(world->getSprite("draftSprite"));
        addObject(up);

        up->body->position = Vec2(120 + i * 300, 50);

        gameLayer->add(up->actor);
    }

    Land* land = new Land();
    addObject(land);

    land->body->position.set(200,100);

    Updraft* up = new Updraft(world->getSprite("draftSprite"));
    addObject(up);

    up->body->position = Vec2(420, 450);

    steamLayer->add(up->actor);
}

void Game::destroy()
{
    delete gameLayer;
    delete steamLayer;
}

void Game::step(double t)
{
    if( t-lastTime > 1.0 / 35.0 )
    {
        universe.handleCollisions();
        universe.move();
    }

    if( t-lastTime > 1.0 / 35.0 )
    {
        for(vector<Object*>::iterator itr = objects.begin(); itr != objects.end(); itr++)
            (*itr)->step();

        if( keySet.find('d') != keySet.end() )
        {
            primaryCraft->body->velocity += Vec2(0.3, 0.0);
            primaryCraft->actor->frame = 2;
        }
        else if( keySet.find('a') != keySet.end() )
        {
            primaryCraft->body->velocity -= Vec2(0.2, 0.0);
            primaryCraft->actor->frame = 1;
        }
        else
        {
            primaryCraft->actor->frame = 0;
        }

        handleCollisions();

        lastTime = t;
    }
}

void Game::drawPhysics() const
{
    universe.draw();
}

void Game::keyDown(unsigned char inkey)
{
    keySet.insert(inkey);
}

void Game::keyUp(unsigned char inkey)
{
    keySet.erase(inkey);
}

void Game::addObject(Object* o)
{
    if( !o->body )
    {
        o->initBody();
    }

    objects.push_back(o);

    if( o->body )
        universe.add(o->body);
}

void Game::handleCollisions()
{
    map<CollisionCode, vector<Object*> > codeMap;
    for(vector<Object*>::iterator itr = objects.begin(); itr != objects.end(); itr++)
    codeMap[(*itr)->collisionCode].push_back(*itr);
    
    vector<Object*>& updrafts(codeMap[kUpdraft]);
    vector<Object*>& crafts(codeMap[kCraft]);

    for( vector<Object*>::iterator uitr = updrafts.begin(); uitr!=updrafts.end(); uitr++ )
    {
        Updraft* updraft = dynamic_cast<Updraft*>(*uitr);

        for( vector<Object*>::iterator citr = crafts.begin(); citr!=crafts.end(); citr++ )
        {
            Craft* craft = dynamic_cast<Craft*>(*citr);

            double v = fabs(updraft->body->position.x - craft->body->position.x);
            double h = craft->body->position.y - updraft->body->position.y;
            
            if( v < 100.0 && h > 0 && h < 200 )
            {
                craft->body->velocity += Vec2(0, 0.4);
            }
        }
    }
}


GameLayer::GameLayer(Game* game)
    : game(game)
{
}

GameLayer::~GameLayer()
{
}

void GameLayer::drawInTree(
    const Mat4& worldMatrix,
    const Color& worldColor) const
{
    const vector<Object*> &objects(game->objects);
    const Universe& universe(game->universe);

    for(vector<Object*>::const_iterator itr = objects.begin();
        itr != objects.end();
        itr++)
    {
        (*itr)->draw();
    }
}


Object::Object()
    : actor(NULL)
    , body(NULL)
{
}

Object::~Object() {}

void Object::initBody()
{
    body = new lib2d::Body(p);
}

void Object::step()
{
}

void Object::draw() const
{
    if( actor && body )
    {
        actor->position = body->position;
        actor->draw();
    }
}


Craft::Craft(Sprite* insprite)
    : Object()
{
    actor = new Actor();
    actor->sprite = insprite;
    collisionCode = kCraft;
}

Craft::~Craft()
{
    delete actor;
}

void Craft::step()
{
    if( body->velocity.y > -4.0 )
        body->velocity-= Vec2(0.0, 0.1);
}

void Craft::initBody()
{
    p.add(-30,-20).add(80,0).add(-30,20);
    body = new lib2d::Body(p);
    body->makeStiff();
}

Updraft::Updraft(Sprite* insprite)
{
    actor = new Actor();
    actor->sprite = insprite;
    collisionCode = kUpdraft;
}

Updraft::~Updraft()
{
    delete actor;
}

void Updraft::step()
{
}

Land::Land()
{
}

Land::~Land()
{
}

void Land::initBody()
{
    p.add(-100,-100).add(100,-100).add(100,100).add(-100,100);
    body = new lib2d::Body(p);

    body->makeImmutable();
}

