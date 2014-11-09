
#ifndef _APP_ENVIRONMENT_
#define _APP_ENVIRONMENT_

#include "app.h"
#include "environment.h"

using namespace g2c;

class AppEnvironment : public Environment {
public:
	AppEnvironment();
	virtual ~AppEnvironment();

private:
	bool touchOn;

protected:
	virtual void enables();

	virtual void step(double t);

	virtual void draw() const;

	virtual void init();
	virtual void keyboard(unsigned char inkey);
	virtual void special(unsigned char inkey);

	virtual void resize(int width, int height);

	virtual void keyDown(unsigned char inkey);
	virtual void keyUp(unsigned char inkey);

	virtual bool mouseDown( const Vec2& C );
	virtual void mouseDragged( const Vec2& C );
	virtual void mouseUp( const Vec2& C );

public:
	App* app;
};

#endif

