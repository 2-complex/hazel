
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


#ifndef _FATTENING_
#define _FATTENING_

#include <string>

#include "graphics.h"
#include "sprites.h"

#include <stdint.h>
#include <vector>


namespace g2c
{
    struct FatteningGeometryInfo
    {
        Geometry* geometry;
        Buffer* buffer;
        IndexBuffer* indexBuffer;
    };

    class Fattening
    {
    public:
        std::vector<double> position;
        std::vector<double> texcoord;
        std::vector<int> indices;

        void clear();
        void add(const Vec2& v, const Vec2& u);
        void add(int a, int b, int c);

        void fatten(const Polygon& polygon, double r);
        FatteningGeometryInfo newGeometry(const std::string& name);
        void populateModel(Model* model, Shape* shape);
    };

} // end namespace


#endif

