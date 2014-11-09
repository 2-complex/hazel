
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


#ifndef _ENVIRONMENT_
#define _ENVIRONMENT_

#include "glutlistener.h"
#include "opengl.h"
#include "util.h"
#include "lin/lin.h"

namespace g2c {

class Environment : public GlutListener
{
friend void fdisplay();
friend void fresize(int w, int h);

public:
    Environment();
    virtual ~Environment();

    /*! If true, calls display over and over*/
    bool animate;

    Mat4 getModelView() const;
    Mat4 getProjection() const;

    int getWindowWidth() const;
    int getWindowHeight() const;


    /*! initWindow Initilaizes glut.  Sets up the gl context, sets flags etc.
        Must be called before the mainLoop.*/
    void initWindow(const char* windowName="Environment",
                  	int windowSizeX=640,
                  	int windowSizeY=480);

    /*  Call this to start the glut main loop, this hands control over to glut
    	to handle events and draw.*/
    void mainLoop();


protected:
    /*! initted says whether the init() function has been called.  Inside display(),
    there is a check to call init once, then */
    mutable bool initted;

    /*! dragging is a variable for use by the click-drag mechanism for moving the
        camera.  When the flag gets set in button, the subsequent mouse-drag and
	mouse-up events are interpreted to pan the camera.*/
    bool dragging;

	/*! While the mouse is getting dragged, this is the location of the previous
		mouse event*/
	Vec2 last;

	/*! flip reports the screen location of the point given coordinates of the
	    form that gl button etc reports them.*/
	Vec2 flip(const Vec2& v) const;

	/*! display gets called by the glut display function.  Think twice before
		overriding if draw() can do the job.  display calls step() and init(),
		so it is not const.*/
	virtual void display();

	/*! button gets called by the like-named glut function.
	    Think twice before overriding if the same job could be done with mouseDown
	    mouseDragged and mouseUp.*/
	virtual void button( int b, int state, int x, int y );

    /*! motion gets called by the like-named glut function.
        Think twice before overriding if the same job could be done with mouseDown
        mouseDragged and mouseUp.*/
    virtual void motion( int x, int y );

    /*!  resize gets called directly by the glut resize function.*/
    virtual void resize( int w, int h );

    /*! gets called by initWindow.  Override to turn things on and off, or set the
        clear-color blending or whathaveyou.*/
    virtual void enables();

	/*! init() is called by display() when the initted flag is set to false.
    then initted gets set true.*/
    virtual void init();

    /*! step() is called by display(), over and over with a time value measured
        in seconds for t.  Override step() to do work to compute what to draw.*/
    virtual void step(double t);

    /*! draw is called after a clear and inside a matrix push.  Override draw()
        to add graphics to the scene using ordinary gl calls like glVertex3f().*/
    virtual void draw() const;

private:
	/*	window height and width updated in the resize function*/
	double windowHeight;
	double windowWidth;

	bool glutInitted;
};

} // end namespace

#endif
