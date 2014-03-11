#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include <stdlib.h>

#include "phys2d.h"

#define DRAW_ON 1

using namespace lib2d;


#if DRAW_ON
#include <glut/glut.h>

void display();
void again();
void reshape(int w, int h);
void motion(int x, int y);
void button( int b, int state, int x, int y );
#endif


#define kNumberOfBoxes 20
#define kNumberOfBalls 20



class Environment
{
    public:
    
    Universe U;
    Polygon P, P1, P2;
    Circle C;
    Body pB, pC, pC2, pB1, pB2;
    Body boxes[kNumberOfBoxes];
    Body balls[kNumberOfBalls];
    Circle C2;
    
    bool freeze_flag;
            
    Environment()
    {
        struct timeval begin_tv;
        buildTrigTable();
        gettimeofday(&begin_tv, NULL);
        srand(begin_tv.tv_usec);
        
        //freeze_flag = true;
        
        P.add(-200,10).add(-200,-10).add(-10,-10).add(-10,-200).add(10,-200).add(10,-10).add(200,-10).add(200,10).add(10,10).add(10,200).add(-10,200).add(-10,10);
        pB.set(P);
        pB.position.set(0.0, 0.0);
        pB.velocity.set(0,0);
        pB.omega = 0.0;
        pB.rotationalInertia = 1000000.0;
        pB.theta = PI/4.0;
        pB.restitution = 0.5;
        pB.name = (char*)"cross";
        U.add(pB);
        
        P2.add(-10,-10).add(10,-10).add(10,10).add(-10,10);
        
        for( int i=0; i<kNumberOfBoxes; i++ )
        {
            boxes[i].set(P2);
            boxes[i].position = Vector2d(0.0, (i+1)*80.0) + Vector2d::randVector(10.0);
            boxes[i].velocity.set(0,-10);
            boxes[i].omega = 0.0;
            boxes[i].rotationalInertia = 1000.0;
            boxes[i].restitution = 0.5;
            boxes[i].theta = 0.0;
            boxes[i].mass = 20.0;
            
            U.add(boxes[i]);
        }
        
        
        for( int i=0; i<kNumberOfBalls; i++ )
        {
            balls[i].set(Circle(0,0, 10.0));
            balls[i].position = Vector2d(50.0, (i+1)*80.0) + Vector2d::randVector(10.0);
            balls[i].velocity.set(0,-10);
            balls[i].omega = 0.0;
            balls[i].rotationalInertia = 1000.0;
            balls[i].restitution = 0.8;
            balls[i].theta = 0.0;
            balls[i].mass = 15.0;
            
            U.add(balls[i]);
        }
        
        
        C.set(0,0,20);
        pC.set(C);
        pC.position.set(130, 400.1);
        pC.omega = 0.0;
        pC.mass = 10.0;
        pC.restitution = 0.9;
        pC.name = (char*)"ball 1";
        
        C2.set(0,0,30);
        pC2.set(C2);
        pC2.position.set(-110, 305.5);
        pC2.velocity.set(0,-10);
        pC2.omega = 0.0;
        pC2.mass = 20.0;
        pC2.rotationalInertia = 2000.0;
        pC2.restitution = 0.1;
        pC2.staticFriction = 3.0;
        pC2.dynamicFriction = 2.0;
        pC2.name = (char*)"ball 2";
        
        pB.nail();
        pB.makeImmutable();
        
        U.add(pC).add(pC2);
        
        U.gravity.set(0,-0.2);
        U.timeStep = 1.0;
    }
    
    void go()
    {
        static int frame = 0;
        
        U.applyGravity();
        
        U.handleCollisions();  //handleCollisions should happen right before move(), because it attempts to hold
                               //invariant that no bodies are about to penetrate eachother
        U.move();
        
        #if DRAW_ON
        U.draw();
        #endif
        
        frame++;
    }
    
    void freeze()
    {
    }
    
};

Environment* gE;


double tv_to_float(struct timeval tv)
{
    return ((double)(tv.tv_sec))  + (((double)(tv.tv_usec)) / 1000000.0);
}


int main( int argc, char** args )
{    
    gE = new Environment;
    
    #if DRAW_ON
    glutInit( &argc, args );
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB );
    
    glutInitWindowPosition(30,30);
    glutInitWindowSize(640,480);
    glutCreateWindow("A Window");
    //glutFullScreen();
    
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(again);
    glutMotionFunc(motion);
    glutMouseFunc(button);
    
    glClearColor(0,0,0,0);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    
    glutMainLoop();
    #else
    
    struct timeval begin_tv, end_tv;
    
    int frames = 1;
    if( argc > 1 )
        frames = atoi(args[1]);
    
    gettimeofday(&begin_tv, NULL);
        
    for( int i=0; i<frames; i++ ) //number of frames of animation
        gE->go();
    
    gettimeofday(&end_tv, NULL);
    
    printf( "test took %f seconds\n", tv_to_float(end_tv) - tv_to_float(begin_tv) );
    
    #endif
        
    return 0;
}



#if DRAW_ON

int gXOffset = 0;
int gYOffset = 0;

void display()
{    
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double clock = ((double)(tv.tv_sec)) + (double)(tv.tv_usec)/1000000.0;
    
    static double lastclock = 0;
    
    if( lastclock + (1.0/30.0) < clock )
    {
        glClear(GL_COLOR_BUFFER_BIT);
        glLoadIdentity();
        gE->go();
        glutSwapBuffers();
        
        lastclock = clock;
    }
}


void reshape(int w, int h)
{
    glViewport( 0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-w/2, w/2, -h/2,h/2);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}


int mode;
int lastx=0;
int lasty=0;

void button( int b, int state, int x, int y )
{
    mode = b;
    lastx = x;
    lasty = y;
    
    exit(0);
}

void motion( int x, int y )
{
    gXOffset += x-lastx;
    gYOffset += y-lasty;
    lastx = x;
    lasty = y;
}

void again()
{
    glutPostRedisplay();
}

#endif

