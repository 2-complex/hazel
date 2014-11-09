
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



#ifndef _ANIMATIONS_
#define _ANIMATIONS_

#include "sprites.h"

namespace g2c {

class MoveVec2 : public Animation {
public:
    MoveVec2(double instart, double induration, Vec2* inv, const Vec2& indst);
    
    Vec2* v;
    Vec2 src;
    Vec2 dst;
    virtual void begin();
    virtual void step(double t);
    virtual void end();
};

class MoveMatrix : public Animation {
public:
    MoveMatrix(double instart, double induration, Mat4* inm, const Mat4& indst);
    
    Mat4* m;
    Mat4 src;
    Mat4 dst;
    virtual void begin();
    virtual void step(double t);
    virtual void end();
};

class WalkInt : public Animation {
public:
    WalkInt(double instart, double induration, int* ina, int infirst, int inlast);
    
    int* a;
    int first;
    int last;
    virtual void begin();
    virtual void step(double t);
    virtual void end();
};

class StopEvents : public Animation {
public:
    StopEvents(double instart, double induration);
    
    virtual void begin();
    virtual void step(double t);
    virtual void end();
};

class Transition : public StopEvents {
public:
    Transition(double instart, double induration, Node* outgoing, Node* incoming);
    
    Node* outgoing;
    Node* incoming;
    
    virtual void begin();
    virtual void step(double t);
    virtual void end();
};

class FadeIn: public Animation {
public:
    FadeIn(double instart, double induration, Node* node, bool inStopsEvents = true);
    
    Node* node;
    
    virtual void begin();
    virtual void step(double t);
    virtual void end();
};

class FadeOut: public Animation {
public:
    FadeOut(double instart, double induration, Node* node, bool inStopsEvents = true);
    
    Node* node;
    
    virtual void begin();
    virtual void step(double t);
    virtual void end();
};

} // end namespace

#endif
