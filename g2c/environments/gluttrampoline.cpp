
#include "gluttrampoline.h"

#include "lin/lin.h"
#include "util.h"

#include "openalplayer.h"

#if __APPLE_CC__
#include "macbank.h"
#else
#include "unixbank.h"
#endif


using namespace std;

GlutTrampoline::GlutTrampoline()
    : app(NULL)
{
}

GlutTrampoline::~GlutTrampoline()
{
}

void GlutTrampoline::enables()
{
    glEnable(GL_TEXTURE_2D);

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

void GlutTrampoline::resize(int w, int h)
{
    Glut::resize(w, h);

    if(app)
        app->resize(w, h);
    else
        g2clog( "GlutTrampoline used with no app\n" );
}

void GlutTrampoline::init()
{
    if(app)
    {
        player = new OpenALPlayer;
	bank = new
#if __APPLE_CC__
            MacFileSystemBank;
#else
            UnixBank;
#endif
        app->setAudioPlayer(player);
        app->setBank(bank);
        app->init();
    }
    else
        g2clog( "GlutTrampoline used with no app\n" );
}

void GlutTrampoline::destroy()
{
    if(app)
    {
        app->destroy();
        delete player;
        delete bank;
    }
    else
        g2clog( "GlutTrampoline used with no app\n" );
}

void GlutTrampoline::step(double t)
{
    if(app)
        app->step(t);
    else
        g2clog( "GlutTrampoline used with no app\n" );
}

void GlutTrampoline::draw() const
{
    if(app)
        app->draw();
    else
        g2clog( "GlutTrampoline used with no app\n" );
}


void GlutTrampoline::keyboard(unsigned char inkey)
{
    if(app)
        app->keyboard(inkey);
    else
        g2clog( "GlutTrampoline used with no app\n" );
}

void GlutTrampoline::special(unsigned char inkey)
{
    if(app)
        app->special(inkey);
    else
        g2clog( "GlutTrampoline used with no app\n" );
}

void GlutTrampoline::keyDown(unsigned char inkey)
{
    if(app)
        app->keyDown(inkey);
    else
        g2clog( "GlutTrampoline used with no app\n" );
}

void GlutTrampoline::keyUp(unsigned char inkey)
{
    if(app)
        app->keyUp(inkey);
    else
        g2clog( "GlutTrampoline used with no app\n" );
}

bool GlutTrampoline::mouseDown(const Vec2& C)
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
        g2clog( "GlutTrampoline used with no app\n" );

    return false;
}

void GlutTrampoline::mouseDragged(const Vec2& C)
{
    if(app)
    {
        if( touchOn )
            app->touchDragged(0, C);
        else
            app->mouseDragged(C);
    }
    else
        g2clog( "GlutTrampoline used with no app\n" );
}

void GlutTrampoline::mouseUp(const Vec2& C)
{
    if(app)
    {
        if( touchOn )
            app->touchUp(0, C);
        else
            app->mouseUp(C);
    }
    else
        g2clog( "GlutTrampoline used with no app\n" );
}
