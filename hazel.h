
#ifndef _TRIPLES_ATTACK_APP_
#define _TRIPLES_ATTACK_APP_

#include "app.h"
#include "sprites.h"
#include "worldbase.h"

#include <map>

using namespace g2c;
using namespace std;


class HazelPrefs : public Serializable
{
public:
	HazelPrefs() : bank(NULL), soundFXOn(true), musicOn(true), unlockedLevel(5)
	{
		addProperty("musicOn", musicOn);
		addProperty("soundFXOn", soundFXOn);
		addProperty("unlockedLevel", unlockedLevel);
	}
	virtual ~HazelPrefs() {}
	
	std::string key;
	Bank* bank;
	
	BoolProperty musicOn;
	BoolProperty soundFXOn;
	IntProperty unlockedLevel;
	
	void rewrite() const;
};

class HazelWorld;
class Touch;



class HazelWorld
	: public WorldBase
{
friend class Hazel;

	Node* getNode(const string& name);

	void initWorld();
};

class Hazel : public App
{
friend class HazelWorld;

private:
	HazelWorld world;
	RendererGL2* renderer;

public:
	Hazel();
	virtual ~Hazel();

	void init();
	void destroy();
	
	virtual void setBank(Bank* inBank);
	virtual void setPlayer(Player* inPlayer);

	virtual void draw() const;
	
	virtual void keyboard(unsigned char inkey);
	virtual void reshape(int width, int height);
    
	virtual bool mouseDown(const Vec2& C);
	virtual void mouseDragged(const Vec2& C);
	virtual void mouseUp(const Vec2& C);
	
	virtual bool touchDown(unsigned int index, const Vec2& C);
	virtual void touchDragged(unsigned int index, const Vec2& C);
	virtual void touchUp(unsigned int index, const Vec2& C);
	
	virtual void step(double t);
};


#endif

