
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


#ifndef _LISTENER_
#define _LISTENER_

#include "lin/lin.h"

#include <set>

namespace g2c {


class Listener;

class Listener {
    
private:
    std::set<unsigned char> keys;
    
public:
    Listener();
    virtual ~Listener();

    /*! button get called by the like-named glut function.
        Think twice before overriding if the same job could be done with mouseUp
        mouseDragged mouseDown.*/
    virtual void button( int b, int state, int x, int y );

    /*! motion gets called by the like-named glut function.
        Think twice before overriding if the same job could be done with mouseUp
        mouseDragged mouseDown.*/
    virtual void motion( int x, int y );
    
    /*! Gets called directly by the glut function keyboard.
        Override to handle key events.*/
    virtual void keyboard(unsigned char inkey);
    
    /*! keyDown gets called when a key is first pressed.
        Override to handle key events.*/
    virtual void keyDown(unsigned char inkey);
    
    /*! keyUp gets called when a key is released.
        Override to handle key events.*/
    virtual void keyUp(unsigned char inkey);

    /*! special gets called directly by the glut function special.
        It should be overridden to handle special keys such as
        arrow keys.*/
    virtual void special(int inkey);
    
    /*! mouseDown() gets called when the mouse button is first depressed.
        Override mouseDown to implement new mouse click behavior.  Return true
        to stop Environment from interpreting the mouse event and panning the
        camera.*/
    virtual bool mouseDown(const Vec2& C);
    
    /*! mouseDragged() gets called when the mouse is dragged.*/
    virtual void mouseDragged(const Vec2& C);

    /*! gets called when the mouse is released.*/
    virtual void mouseUp(const Vec2& C);
 
    bool listening;
 
    /*! The base-class behavior is to appeal to the delegate if it's non-null
        for each virtual function.*/
    Listener* delegate;
};

} // end namespace


#endif
