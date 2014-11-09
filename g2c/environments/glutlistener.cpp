
/*
  g2c Copyright (C) 2012 2-Complex

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/



#include "glutlistener.h"
#include "opengl.h"


namespace g2c {

GlutListener* gGlutListener = NULL;

GlutListener::GlutListener() : listening(true), delegate(NULL) {}
GlutListener::~GlutListener() {}

void GlutListener::button(int b, int state, int x, int y)
{
    if( delegate )
        delegate->button(b, state, x, y);
}

void GlutListener::motion(int x, int y)
{
    if( delegate )
        delegate->motion(x, y);
}

void GlutListener::keyboard(unsigned char inkey)
{
    if( delegate )
        delegate->keyboard(inkey);
}

void GlutListener::keyDown(unsigned char inkey)
{
    if( delegate )
        delegate->keyDown(inkey);
}

void GlutListener::keyUp(unsigned char inkey)
{
    if( delegate )
        delegate->keyUp(inkey);
}

void GlutListener::special(int inkey)
{
    if( delegate )
        delegate->special(inkey);
}

bool GlutListener::mouseDown(const Vec2& C)
{
    if( delegate )
        return delegate->mouseDown(C);
    return false;
}

void GlutListener::mouseDragged(const Vec2& C)
{
    if( delegate )
        delegate->mouseDragged(C);
}

void GlutListener::mouseUp(const Vec2& C)
{
    if( delegate )
        delegate->mouseUp(C);
}

void fbutton( int b, int state, int x, int y )
{
    gGlutListener->button(b, state, x, y);
    glutPostRedisplay();
}

void fkeyboard(unsigned char inkey, int x, int y)
{
    if( gGlutListener->keys.find(inkey)==gGlutListener->keys.end() )
    {
        gGlutListener->keyDown(inkey);
        gGlutListener->keys.insert(inkey);
    }
    
    gGlutListener->keyboard(inkey);
    glutPostRedisplay();
}

void fkeyboardUp(unsigned char inkey, int x, int y)
{
    gGlutListener->keys.erase(inkey);
    
    gGlutListener->keyUp(inkey);
    glutPostRedisplay();
}

void fspecial(int inkey, int x, int y)
{
    gGlutListener->special(inkey);
    glutPostRedisplay();
}

void fmotion(int x, int y)
{
    gGlutListener->motion(x,y);
    glutPostRedisplay();
}

void setGlobalGlutListener(GlutListener* glutListener)
{
    gGlutListener = glutListener;
}

void initGlutListener()
{
    glutMotionFunc(fmotion);
    glutMouseFunc(fbutton);
    glutKeyboardFunc(fkeyboard);
    glutKeyboardUpFunc(fkeyboardUp);
    glutSpecialFunc(fspecial);
}

}

