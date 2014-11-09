
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



#include "listener.h"
#include "opengl.h"


namespace g2c {

Listener* gListener = NULL;

Listener::Listener() : listening(true), delegate(NULL) {}
Listener::~Listener() {}

void Listener::button(int b, int state, int x, int y)
{
    if( delegate )
        delegate->button(b, state, x, y);
}

void Listener::motion(int x, int y)
{
    if( delegate )
        delegate->motion(x, y);
}

void Listener::keyboard(unsigned char inkey)
{
    if( delegate )
        delegate->keyboard(inkey);
}

void Listener::keyDown(unsigned char inkey)
{
    if( delegate )
        delegate->keyDown(inkey);
}

void Listener::keyUp(unsigned char inkey)
{
    if( delegate )
        delegate->keyUp(inkey);
}

void Listener::special(int inkey)
{
    if( delegate )
        delegate->special(inkey);
}

bool Listener::mouseDown(const Vec2& C)
{
    if( delegate )
        return delegate->mouseDown(C);
    return false;
}

void Listener::mouseDragged(const Vec2& C)
{
    if( delegate )
        delegate->mouseDragged(C);
}

void Listener::mouseUp(const Vec2& C)
{
    if( delegate )
        delegate->mouseUp(C);
}

} // end namespace

