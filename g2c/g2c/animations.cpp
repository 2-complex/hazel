
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


#include "animations.h"

namespace g2c {

StopEvents::StopEvents(double instart, double induration) :
    Animation(instart, induration)
{
    stopsEvents = true;
}

void StopEvents::begin()
{
    
}

void StopEvents::step(double t)
{
    
}

void StopEvents::end()
{
    
}

MoveMatrix::MoveMatrix(double instart, double induration, Mat4* inm, const Mat4& indst) :
    Animation(instart, induration), m(inm), dst(indst) {}

void MoveMatrix::begin()
{
    src = *m;
}

void MoveMatrix::step(double t)
{
    t = max(0, min(1, t));
    t = t*t*(3.0-2.0*t);
    *m = (1.0-t) * src + t * dst;
}

void MoveMatrix::end()
{
    *m = dst;
}

MoveVec2::MoveVec2(double instart, double induration, Vec2* inv, const Vec2& indst) :
    Animation(instart, induration), v(inv), dst(indst) { }

void MoveVec2::begin()
{
    src = *v;
}

void MoveVec2::step(double t)
{
    *v = (1.0-t) * src + t * dst;
}

void MoveVec2::end()
{
    *v = dst;
}


WalkInt::WalkInt(double instart, double induration, int* ina, int infirst, int inlast) :
    Animation(instart, induration), a(ina), first(infirst), last(inlast) { }

void WalkInt::begin()
{
    *a = first;
}

void WalkInt::step(double t)
{
    if(last < first)
        t = 1.0-t;
    int m = min(last, first);
    int M = max(last, first);
    int d = M-m+1;
    int k = d*t;
    *a = m+k;
}

void WalkInt::end()
{
    *a = last;
}




Transition::Transition(double instart, double induration, Node* outgoing, Node* incoming) :
    StopEvents(instart, induration), outgoing(outgoing), incoming(incoming)
{
    
}

void Transition::begin()
{
    if(incoming)
        incoming->visible = false;
    if(outgoing)
        outgoing->color.a = 1.0;
}

void Transition::step(double t)
{
    if( t < 0.5 )
    {
        if(incoming)
        {
            incoming->visible = false;
        }
        
        if(outgoing)
        {
            outgoing->visible = true;
            outgoing->color.a = 1.0 - 2.0 * t;
        }
    }
    else
    {
        if(incoming)
        {
            incoming->visible = true;
            incoming->color.a = 2.0 * t - 1.0;
        }
        
        if(outgoing)
        {
            outgoing->visible = false;
        }
    }
}

void Transition::end()
{
    if(outgoing)
    {
        outgoing->visible = false;
    }
    
    if(incoming)
    {
        incoming->visible = true;
        incoming->color.a = 1.0;
    }
}

FadeIn::FadeIn(double start,
               double duration,
               Node* node,
               bool inStopsEvents) :
    Animation(start, duration),
    node(node)    
{
    stopsEvents = inStopsEvents;
}

void FadeIn::begin()
{
    node->visible = true;
    node->color.a = 0.0;
}

void FadeIn::step(double t)
{
    node->color.a = t;
}

void FadeIn::end()
{
    node->visible = true;
    node->color.a = 1.0;
}

FadeOut::FadeOut(double instart,
                 double induration,
                 Node* node,
                 bool inStopsEvents) :
    Animation(instart, induration),
    node(node)
{
    stopsEvents = inStopsEvents;
}

void FadeOut::begin()
{
    node->visible = true;
    node->color.a = 1.0;
}

void FadeOut::step(double t)
{
    node->color.a = 1.0-t;
}

void FadeOut::end()
{
    node->visible = false;
    node->color.a = 0.0;
}

} // end namespace
