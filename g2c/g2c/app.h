
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


#ifndef _APP_
#define _APP_

#include "lin/lin.h"

#include "bank.h"
#include "audioplayer.h"

namespace g2c {

/*!App is part of a cross platform development strategy.  To write a cross-platform application,
   subclass App and override functions to handle events.  If you use portable code, the app can
   be instantiated and used by platform-specific so-called trampoline project on a variety of
   platforms.*/
class App {
public:
    App();
    virtual ~App();

    virtual void setBank(Bank*);
    virtual void setAudioPlayer(AudioPlayer*);

    /*! init() gets called once at the beginning of the Apps life.  The trampoline is required
        to make an OpenGL context current at the time of calling init().*/
    virtual void init();

    /*! step() gets called once per frame of animation, but the OpenGL context is
        not required to be current.  The argument is an absolute time measured in seconds.*/
    virtual void step(double t);

    /*! draw() gets called once per frame of animation, the OpenGL context is required to be current.*/
    virtual void draw() const;

    /*! destroy() gets called at the end of the Apps life before it gets deleted.*/
    virtual void destroy();

    /*! Override keyDown() to handle keyboard events.  keyDown() gets called the moment a key is depressed.
        NOT repeatedly after that.*/
    virtual void keyDown(unsigned char inkey);

    /*! Override keyUp to handle keyboard events.  keyUp() gets called the moment a key is released.*/
    virtual void keyUp(unsigned char inkey);

    /*! Override mouseDown() to handle mouse events.  mouseDown() gets called when the left mouse
        button is pressed or, on a touch device, when the user taps down on the screen.*/
    virtual bool mouseDown(const Vec2& C);

    /*! Override mosueDragged() to handle mouse events.  mouseDragged() gets called when the left
        mouse button is pressed and the user moves the mouse or, on a touch screen device, when the
        user taps down and drags.*/
    virtual void mouseDragged(const Vec2& C);

    /*! Override mouseUp() to handle mouse events.  mouseUp() gets called when the left mouse button
        is released or, on a touch screen device, when the user releases a touch.*/
    virtual void mouseUp(const Vec2& C);

    /*! Override touchDown() to handle multi-touch events.  touchDown() gets called when a touch-
        and-drag begins, future calls to touchDragged will be called with the same index for that
        continuous touch-and-drag motion.*/
    virtual bool touchDown(unsigned int index, const Vec2& C);

    /*! Override touchDragged() to handle multi-touch events.  touchDragged gets called whenever
        the user moves a finger on the touch screen.*/
    virtual void touchDragged(unsigned int index, const Vec2& C);

    /*! Override touchUp() to handle multi-touch events.   touchUp() gets called when the user
        removes a finger from the touch screen.*/
    virtual void touchUp(unsigned int index, const Vec2& C);

    /*! Override controllerKeyDOwn() to handle a key press on a game controller.  Function is called
        once when a key is pressed only, NOT repeatedly after that.*/
    virtual void controllerKeyDown(int deviceId, char c);

    /*! Override controllerKeyUp() to handle a key press on a game controller.  Function is called
        once when a key is released.*/
    virtual void controllerKeyUp(int deviceId, char c);

    /*! Override controllerMotion to handle the motion of a stick on a game controller.  If there
        are multiple controllers, the argument deviceId indicates the controller issuing the event.
        If there are multiple sticks on the controller the argument inputId indicates which stick.*/
    virtual void controllerMotion(int deviceId, int inputId, double x, double y);

    /*! Override controllerTriggerMotion to handle the motion of the analog triggers on a game
        controller.  If there are multiple controllers, the argument deviceId indicates the
        controller issuing the event.  If there are multiple triggers, the argument inputId
        indicates which trigger.*/
    virtual void controllerTriggerMotion(int deviceId, int inputId, double x);

    virtual void keyboard(unsigned char inkey);
    virtual void special(int inkey);

    virtual void resize(int width, int height);

    bool getNeedsKeyboard();
    void finishTyping();

protected:
    void showKeyboard();
    void hideKeyboard();

private:
    bool needsKeyboard;
};

} // end namespace


#endif

