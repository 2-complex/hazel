
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


#ifndef _PAN_ENVIRONMENT_
#define _PAN_ENVIRONMENT_

#include "environment.h"
#include "lin/lin.h"

using namespace g2c;

class PanEnvironment : public Environment {
public:
	PanEnvironment();
	virtual ~PanEnvironment();
	
protected:
	virtual void display();
	virtual void draw() const;
	virtual void motion(int x, int y);
	virtual void keyboard(unsigned char inkey);
	virtual void enables();
    
    /*	drawAxes is a utility function to draw the x, y and z
	    axes in red green blue respectively.*/
	void drawAxes() const;
	
	/*  draw2d is called with a different Mat4 on the stack from draw.
        It's for drawing in 2d in pixel coordinates on the screen, like for a
        ui or something.*/
	virtual void draw2d() const;
    
	/*	getCameraLoc returns the current camera location in 3-space*/
	Vec3 getCameraLoc() const;
	Vec3 getLookAtLoc() const;
	
	/*	lookAt sets the camera's lookAt point*/
	void lookAt(const Vec3& V);
    
public:
	/*	These angles describe the current camera position, the 3-space
		coordinates of the camera should be accessed using the accessor
		function getCameraLoc().*/
	double cameraTheta;
	double cameraPhi;
	double cameraRadius;
			
	/*  The point where the camera is aimed and about which the camera rotates.
	    Set using the accessort lookAt().*/
	Vec3 cameraLookAtLoc;
	
	/*	firstX, firstY, firstTheta and firstPhi are member variables for use by
	    the click-drag mechanism to move the camera, but even if the camera is
	    not moving (i.e. if mouseDown returns true), firstX and firstY still
	    get set.*/
	int firstX;
	int firstY;
	double firstTheta;
	double firstPhi;
};

#endif
