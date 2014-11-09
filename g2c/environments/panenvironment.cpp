
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


#include "panenvironment.h"

#include "lin/lin.h"
#include "util.h"

using namespace std;


PanEnvironment::PanEnvironment()
{
    cameraRadius = 50;
    cameraPhi = .5;
    cameraTheta = -0.25;

    firstX=0;
    firstY=0;
    firstTheta=0;
    firstPhi=0;
}

PanEnvironment::~PanEnvironment() {}

void PanEnvironment::display()
{
    if( !initted )
    {
        init();
        initted = true;
    }

    step( currentTime() );

    glEnable(GL_DEPTH_TEST);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    int w = getWindowWidth();
    int h = getWindowHeight();

    gluPerspective(
        40.0, /* field of view in degree */
        1.0*w/h, /* aspect ratio */
        0.1*cameraRadius, /* Z near */
        1000*cameraRadius ); /* Z far */

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    Vec3 cLoc(getCameraLoc());
    Vec3 lLoc(getLookAtLoc());
    gluLookAt( cLoc.x, cLoc.y, cLoc.z,  lLoc.x, lLoc.y, lLoc.z,  0, 0, 1 );

    draw();

    glPopMatrix();


    /*draw in 2d on the screen*/

    glDisable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0.0, w, 0.0, h);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    draw2d();

    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopMatrix();

    glutSwapBuffers();

    if( animate )
        glutPostRedisplay();
}


void PanEnvironment::draw() const
{
    drawAxes();
}


void PanEnvironment::drawAxes() const
{
    glPushAttrib(GL_LIGHTING);
    glDisable(GL_LIGHTING);

    // draw axes
    glBegin(GL_LINES);
    double r = 10;
    glColor3f(1,0,0);
    glVertex3f(-r,0,0);
    glVertex3f( r,0,0);
    glColor3f(0,1,0);
    glVertex3f(0,-r,0);
    glVertex3f(0, r,0);
    glColor3f(0,0,1);
    glVertex3f(0,0,-r);
    glVertex3f(0,0, r);
    glEnd();

    glPopAttrib();
}


void PanEnvironment::motion(int x, int y)
{
    Vec2 c = flip(Vec2(x,y));

    if( dragging )
    {
        Vec2 delta = -0.01*(c - last);
        cameraTheta += delta.x;
        cameraPhi += delta.y;

        if(cameraPhi > 0.5*M_PI) cameraPhi = 0.5*M_PI;
        if(cameraPhi < -0.5*M_PI) cameraPhi = -0.5*M_PI;

        last = c;
    }

    mouseDragged(c);
}


void PanEnvironment::keyboard(unsigned char inkey)
{
    switch(inkey)
    {
        case '+':
        case '=':
            cameraRadius *= 0.9;
        break;

        case '_':
        case '-':
            cameraRadius /= 0.9;
        break;

        default:
        break;
    }
}


void PanEnvironment::enables()
{
    glClearColor(0,0,0,1);
    glEnable(GL_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glCullFace(GL_NONE);
}

void PanEnvironment::draw2d() const
{

}

Vec3 PanEnvironment::getCameraLoc() const
{
    return cameraLookAtLoc+
        cameraRadius*Vec3(cos(cameraTheta)*cos(cameraPhi),
                              sin(cameraTheta)*cos(cameraPhi),
                              sin(cameraPhi));
}

void PanEnvironment::lookAt(const Vec3& V)
{
    cameraLookAtLoc = V;
}

Vec3 PanEnvironment::getLookAtLoc() const
{
    return cameraLookAtLoc;
}

