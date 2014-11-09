
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


#include "environment.h"

#include "util.h"
#include "lin/lin.h"

void display();
void resize(int w, int h);
void motion(int x, int y);
void button(int b, int state, int x, int y);
void keyboard(unsigned char inkey, int x, int y);

namespace g2c {

Environment* gEnvironment = NULL;

Environment::Environment()
    : dragging(false)
    , initted(false)
    , glutInitted(false)
    , animate(true)
{
}

Environment::~Environment()
{
}

Mat4 Environment::getModelView() const
{
    double m[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, m);
    Mat4 M;
    M.set(m);
    return M;
}

Mat4 Environment::getProjection() const
{
    double m[16];
    glGetDoublev(GL_PROJECTION_MATRIX, m);
    Mat4 M;
    M.set(m);
    return M;
}

int Environment::getWindowWidth() const
{
    return windowWidth;
}

int Environment::getWindowHeight() const
{
    return windowHeight;
}


Vec2 Environment::flip(const Vec2& v) const
{
    return Vec2(v.x, windowHeight - v.y);
}

void Environment::display()
{
    if( !initted )
    {
        initOpenGL();
        init();
        initted = true;
    }

    step( currentTime() );

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    draw();

    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glutSwapBuffers();

    if( animate )
        glutPostRedisplay();
}

void Environment::resize(int w, int h)
{
    windowHeight = h;
    windowWidth = w;

    if( !initted )
    {
        initOpenGL();
        init();
        initted = true;
    }

    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
}


void Environment::button( int b, int state, int x, int y )
{
    Vec2 c = flip(Vec2(x,y));

    if( state == 0 ) // button down
    {
        if( !mouseDown(c) )
        {
            last = c;
            dragging = true;
        }
    }

    if( state == 1 ) // button up
    {
        mouseUp(c);
        dragging = false;
    }
}

void Environment::motion(int x, int y)
{
    Vec2 c = flip(Vec2(x,y));
    mouseDragged(c);

    if( dragging )
    {
        last = c;
    }
}


void fdisplay() { gEnvironment->display(); }
void fresize(int w, int h) { gEnvironment->resize(w,h); }


void Environment::initWindow(
    const char* name,
    int width,
    int height)
{
    int argc = 0;
    char** args = NULL;

    // initilaize glut
    glutInit( &argc, args );
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );

    // bring up a window called windowName
    glutInitWindowSize( width, height );
    glutCreateWindow( name );

    // call the virtual function in case somebody wants more enables
    enables();

    glutInitted = true;
}


void Environment::enables()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);

    glCullFace(GL_NONE);

    glEnable(GL_ALPHA);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}


void Environment::mainLoop()
{
    if( !glutInitted )
    {
        initWindow();
    }

    glutInitted = true;

    gEnvironment = this;
    setGlobalGlutListener(this);
    initGlutListener();

    // install the functions in this file as callbacks
    glutDisplayFunc(fdisplay);
    glutReshapeFunc(fresize);

    // leave the rest to glut
    glutMainLoop();
}

void Environment::init()
{
}

void Environment::step(double /*t*/)
{
    animate = false;
}

void Environment::draw() const
{
}

}

