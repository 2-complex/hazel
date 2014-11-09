
#include "appenvironment.h"

#include "lin/lin.h"
#include "util.h"

using namespace std;

AppEnvironment::AppEnvironment() : app(NULL) {}
AppEnvironment::~AppEnvironment() {}

void AppEnvironment::enables()
{
    glEnable(GL_TEXTURE_2D);

    glEnable(GL_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);                        // enables a light (we need that aparently)
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 0);  // sets lighting to one-sided

    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_RGB, GL_MODULATE); // ?

    glCullFace(GL_NONE);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}

void AppEnvironment::resize(int w, int h)
{
	Environment::resize(w, h);

	if(app)
		app->resize(w, h);
	else
		g2clog( "AppEnvironment used with no app\n" );
}

void AppEnvironment::init()
{
	if(app)
		app->init();
	else
		g2clog( "AppEnvironment used with no app\n" );
}

void AppEnvironment::step(double t)
{
	if(app)
		app->step(t);
	else
		g2clog( "AppEnvironment used with no app\n" );
}

void AppEnvironment::draw() const
{
	if(app)
		app->draw();
	else
		g2clog( "AppEnvironment used with no app\n" );
}


void AppEnvironment::keyboard(unsigned char inkey)
{
	if(app)
		app->keyboard(inkey);
	else
		g2clog( "AppEnvironment used with no app\n" );
}

void AppEnvironment::special(unsigned char inkey)
{
	if(app)
		app->special(inkey);
	else
		g2clog( "AppEnvironment used with no app\n" );
}

void AppEnvironment::keyDown(unsigned char inkey)
{
	if(app)
		app->keyDown(inkey);
	else
		g2clog( "AppEnvironment used with no app\n" );
}

void AppEnvironment::keyUp(unsigned char inkey)
{
	if(app)
		app->keyUp(inkey);
	else
		g2clog( "AppEnvironment used with no app\n" );
}

bool AppEnvironment::mouseDown(const Vec2& C)
{
	if(app)
	{
		touchOn = app->touchDown(0, C);
		if( !touchOn )
		{
			return app->mouseDown(C);
		}
		return touchOn;
	}
	else
		g2clog( "AppEnvironment used with no app\n" );

	return false;
}

void AppEnvironment::mouseDragged(const Vec2& C)
{
	if(app)
	{
		if( touchOn )
			app->touchDragged(0, C);
		else
			app->mouseDragged(C);
	}
	else
		g2clog( "AppEnvironment used with no app\n" );
}

void AppEnvironment::mouseUp(const Vec2& C)
{
	if(app)
	{
		if( touchOn )
			app->touchUp(0, C);
		else
			app->mouseUp(C);
	}
	else
		g2clog( "AppEnvironment used with no app\n" );
}
