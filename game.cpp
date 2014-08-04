
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
    primaryCraft = new Craft(world->getSampler("hazelSprite"));
    addObject(primaryCraft);

    primaryCraft->body->position.set(100, 1000);

    Layer* gameContainer = (Layer*)world->findChild("gameContainer");

    gameLayer = new GameLayer(this);
    steamLayer = new Layer;

    gameContainer->add(gameLayer);
    gameContainer->add(steamLayer);

    for( int i = 0; i < 3; i++ )
    {
        Updraft* up = new Updraft(world->getSampler("draftSprite"));
        addObject(up);

        up->body->position = Vec2(120 + i * 500, 50);
    }

    Land* land = new Land();
    addObject(land);
    land->body->position.set(200,100);

    Updraft* up = new Updraft(world->getSampler("draftSprite"));
    addObject(up);

    up->body->position = Vec2(420, 450);


    {
        Tree* tree = new Tree(world->getSampler("GB_0"));
        addObject(tree);
        tree->body->position.set(150,20);
    }

    {
        Tree* tree = new Tree(world->getSampler("GB_1"));
        addObject(tree);
        tree->body->position.set(650,20);
    }

    {
        Tree* tree = new Tree(world->getSampler("TB_1"));
        addObject(tree);
        tree->body->position.set(1250,150);
    }

    {
        Tree* tree = new Tree(world->getSampler("TT_1"));
        addObject(tree);
        tree->body->position.set(1150,550);
    }

    landModel.bank = world->bank;
    world->bank->initSerializableWithPath(&landModel, "models/land.model");
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
            
            if( v < 100.0 && h > 0 && h < 400 )
            {
                craft->body->velocity += Vec2(0, 0.25);
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
//    const Universe& universe(game->universe);

    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

    for(vector<Object*>::const_iterator itr = objects.begin();
        itr != objects.end();
        itr++)
    {
        (*itr)->draw();
    }

    glBlendFunc(GL_ONE, GL_ONE);

    for(vector<Object*>::const_iterator itr = objects.begin();
        itr != objects.end();
        itr++)
    {
        (*itr)->drawTranslucent();
    }

    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
}


Object::Object()
    : actor(NULL)
    , body(NULL)
{
}

Object::~Object() {}

void Object::initBody()
{
    /*
    if( actor && actor->sprite )
    {
        
        vector<Vec2> L( actor->sprite->polygon.getVertices() );

        for(vector<Vec2>::iterator itr = L.begin();
            itr != L.end();
            itr++)
        {
            p.add(*itr);
        }
    }
    */
    
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

void Object::drawTranslucent() const
{
}


Craft::Craft(Sampler* insampler)
    : Object()
{
    actor = new Actor();
    actor->sprite = insampler;
    collisionCode = kCraft;
}

Craft::~Craft()
{
    delete actor;
}

void Craft::step()
{
    if( body->velocity.y > -8.0 )
        body->velocity-= Vec2(0.0, 0.15);
}

void Craft::initBody()
{
    Object::initBody();
    body->makeStiff();
}

Updraft::Updraft(Sampler* insampler)
{
    actor = new Actor();
    actor->sprite = insampler;
    collisionCode = kUpdraft;
}

Updraft::~Updraft()
{
    delete actor;
}

void Updraft::step()
{
}

void Updraft::draw() const
{
}

void Updraft::drawTranslucent() const
{
    actor->position = body->position + Vec2(0, 250);
    actor->draw();
}

Land::Land()
{
}

Land::~Land()
{
}

void Land::initBody()
{
    p.add(-1000, -100).add(1000, -100).add(1000, 0).add(-1000, 0);
    body = new lib2d::Body(p);
    body->makeImmutable();

    vector<Vec2> L(p.L);



/*
    for(vector<Vec2>::iterator itr = L.begin();
        itr != L.end();
        itr++)
    {
        
    }
*/
}

Tree::Tree(Sampler* insampler)
{
    actor = new Actor();
    actor->sprite = insampler;
    collisionCode = kTree;
}

Tree::~Tree()
{
    delete actor;
}

void Tree::initBody()
{
    Object::initBody();
    body->makeImmutable();
}


