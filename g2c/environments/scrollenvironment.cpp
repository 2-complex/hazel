
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


#include "scrollenvironment.h"

#include "util.h"
#include "lin/lin.h"


ScrollEnvironment::ScrollEnvironment() {}
ScrollEnvironment::~ScrollEnvironment() {}

void ScrollEnvironment::resize(int w, int h)
{
	Environment::resize(w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0, w, 0.0, h);
	glTranslatef(center.x, center.y, 0.0);
}

void ScrollEnvironment::display()
{
	if( !initted )
	{
		init();
		initted = true;
	}

	step( currentTime() );

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    draw();
    glPopMatrix();

    glutSwapBuffers();

    if(animate)
    	glutPostRedisplay();
}

void ScrollEnvironment::motion(int x, int y)
{
	Vec2 c = flip(Vec2(x,y));

	if( dragging )
	{
		center += c - last;
		last = c;
	}

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	int w = getWindowWidth();
	int h = getWindowHeight();
	gluOrtho2D(0.0, w, 0.0, h);
	glTranslatef(center.x, center.y, 0.0);

	mouseDragged(c);
}


