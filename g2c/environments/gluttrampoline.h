
#ifndef _GLUT_TRAMPOLINE_
#define _GLUT_TRAMPOLINE_

#include "app.h"
#include "glut.h"
#include "environment.h"

#include "bank.h"
#include "audioplayer.h"

using namespace g2c;

class GlutTrampoline : public Glut
{
public:
	GlutTrampoline();
	virtual ~GlutTrampoline();

private:
	bool touchOn;
	Bank* bank;
	AudioPlayer* player;

protected:
	virtual void enables();

	virtual void step(double t);

	virtual void draw() const;

	virtual void init();
        virtual void destroy();

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

