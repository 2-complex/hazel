
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


Hazel::Hazel()
	: lastTime(0.0)
{

}

Hazel::~Hazel()
{

}

void Hazel::setBank(Bank* inBank)
{
    world.bank = inBank;
}

void Hazel::setPlayer(Player* inPlayer)
{
    world.initSound(inPlayer);
}

void Hazel::init()
{
    renderer = new RendererGL2;
    Sprite::renderer = renderer;
    renderer->init();

    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_ALPHA);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    world.initWithPath("hazel.world");
    world.init();
    world.initWorld();

    primaryCraft = new Craft;
    primaryCraft->actor = world.hazel;
    primaryCraft->position.set(100, 300);

    objects.push_back(primaryCraft);

    for( int i = 0; i < 3; i++ )
    {
	    Updraft* up = new Updraft(world.getSprite("draftSprite"));
	    up->position = Vec2(120 + i * 300, 50);
		objects.push_back(up);
		world.add(up->actor);
	}
}

void Hazel::destroy()
{
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

    for(vector<Object*>::const_iterator itr = objects.begin(); itr != objects.end(); itr++)
		(*itr)->draw();

    world.draw();
}

void Hazel::keyboard(unsigned char inkey)
{

}

void Hazel::keyDown(unsigned char inkey)
{
    keySet.insert(inkey);
}

void Hazel::keyUp(unsigned char inkey)
{
    keySet.erase(inkey);
}

void Hazel::reshape(int width, int height)
{
    renderer->projection = orthographic(0, width, 0, height, -1, 1);
}

void Hazel::step(double t)
{
    if( t-lastTime > 1.0 / 35.0 )
    {
        for(vector<Object*>::iterator itr = objects.begin(); itr != objects.end(); itr++)
            (*itr)->step();

        if( keySet.find('d') != keySet.end() )
        {
            primaryCraft->velocity += Vec2(0.3, 0.0);
            primaryCraft->actor->frame = 2;
        }
        else if( keySet.find('a') != keySet.end() )
       	{
            primaryCraft->velocity -= Vec2(0.2, 0.0);
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

void Hazel::handleCollisions()
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

			double v = fabs(updraft->position.x - craft->position.x);
			double h = craft->position.y - updraft->position.y;
			
			if( v < 100.0 && h > 0 && h < 200 )
			{
				craft->velocity += Vec2(0, 0.2);
			}
		}
	}
	printf( "\n" );
}

Object::Object()
	: actor(NULL)
{
}

Object::~Object() {}

void Object::step()
{
    if( velocity.y > -4.0 )
        velocity-= Vec2(0.0, 0.1);

    position += velocity;
}

void Object::draw() const
{
	actor->position = position;
}


Craft::Craft()
{
	collisionCode = kCraft;
}

Craft::~Craft()
{

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


