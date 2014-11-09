
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


#include "app.h"

namespace g2c {

App::App()
{
}

App::~App()
{
}

void App::setBank(Bank*)
{

}

void App::setAudioPlayer(AudioPlayer*)
{

}

void App::init()
{
}

void App::step(double /*t*/)
{
}

void App::draw() const
{
}

void App::destroy()
{
}

void App::keyboard(unsigned char /*inkey*/)
{
}

void App::resize(int /*width*/, int /*height*/)
{
}

void App::special(int /*inkey*/)
{
}

void App::keyDown(unsigned char /*inkey*/)
{
}

void App::keyUp(unsigned char /*inkey*/)
{
}

bool App::mouseDown(const Vec2& /*C*/)
{
    return false;
}

void App::mouseDragged(const Vec2& /*C*/)
{
}

void App::mouseUp(const Vec2& /*C*/)
{
}

bool App::touchDown(unsigned int index, const Vec2& C)
{
    return false;
}

void App::touchDragged(unsigned int index, const Vec2& C)
{
}

void App::touchUp(unsigned int index, const Vec2& C)
{
}

void App::controllerKeyDown(int deviceId, char c)
{

}

void App::controllerKeyUp(int deviceId, char c)
{

}

void App::controllerMotion(int deviceId, int inputId, double x, double y)
{

}

void App::controllerTriggerMotion(int deviceId, int inputId, double x)
{

}

bool App::getNeedsKeyboard()
{
    return needsKeyboard;
}

void App::showKeyboard()
{
    needsKeyboard = true;
}

void App::hideKeyboard()
{
    needsKeyboard = false;
}

void App::finishTyping()
{
    needsKeyboard = false;
}

} // end namespace

