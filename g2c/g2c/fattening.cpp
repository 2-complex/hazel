
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


#include "fattening.h"

using namespace std;

namespace g2c
{

void Fattening::clear()
{
    position.clear();
    texcoord.clear();
    indices.clear();
}

void Fattening::add(const Vec2& v, const Vec2& u)
{
    position.push_back(v.x);
    position.push_back(v.y);
    position.push_back(0.0);

    texcoord.push_back(u.x);
    texcoord.push_back(u.y);
}

void Fattening::add(int a, int b, int c)
{
    indices.push_back(a);
    indices.push_back(b);
    indices.push_back(c);
}

void Fattening::fatten(
    const g2c::Polygon& polygon,
    double r)
{
    vector<Vec2> polygonVertices = polygon.getVertices();
    int n = polygonVertices.size();

    for( int i = 0; i < n; i++ )
        add(polygonVertices[i], Vec2(0,0));

    vector<int> vpv;
    int outerVertexCount = 0;

    for( int i = 0; i < n; i++ )
    {
        int a = (i+n-1)%n;
        int b = (i)%n;
        int c = (i+1)%n;

        Vec2 p = polygonVertices[b];
        Vec2 u = (p - polygonVertices[a]);
        Vec2 v = (p - polygonVertices[c]);

        u = u / u.mag();
        v = v / v.mag();

        double d = (u.x * v.y - u.y * v.x);

        if( d >= 0.0 )
        {
            Vec2 m = u + v;
            double amp = (1.0 / sqrt(.5*(1.0 - u.dot(v))));
            m = amp * m / m.mag();

            Vec2 m0 = Vec2(u.y, -u.x);
            Vec2 m1 = Vec2(-v.y, v.x);

            add( p - r * m, m0 );
            add( p - r * m, m1 );
            vpv.push_back(2);
            outerVertexCount += 2;
        }
        else
        {
            Vec2 m0 = Vec2(u.y, -u.x);
            Vec2 m1 = Vec2(-v.y, v.x);

            double omega = acos(m0.dot(m1));

            int n = omega * 2; // segments per radian
            if( n < 2 )
                n = 2;

            for( int i = 0; i <= n; i++ )
            {
                double t = 1.0 * i / n;
                Vec2 m = sin((1.0-t)*omega) / sin(omega) * m0
                    + sin(t*omega) / sin(omega) * m1;
                m = m / m.mag();
                add(p + r * m, m);
            }

            vpv.push_back(n+1);
            outerVertexCount += n+1;
        }
    }

    int i = 0, k = 0;

    for( vector<int>::iterator itr = vpv.begin(); itr!=vpv.end(); itr++ )
    {
        int count = *itr;

        for( int j = 0; j < count-1; j++ )
        {
            add(i, n+(k%outerVertexCount), n+((k+1)%outerVertexCount));
            k++;
        }

        add(i, n+k, n+(k+1)%outerVertexCount);
        add(i, n+(k+1)%outerVertexCount, (i+1)%n);

        k++;
        i++;
    }

    vector<int> ti = polygon.getTriangleIndices();

    for( int i = 0; i < ti.size() / 3; i++ )
    {
        add(ti[3*i + 0],
            ti[3*i + 1],
            ti[3*i + 2]);
    }
}

FatteningGeometryInfo Fattening::newGeometry(const string& name)
{
    vector<double> v;

    int positionCount = position.size() / 3;

    for( int i = 0; i < positionCount; i++ )
    {
        v.push_back(position[3*i + 0]);
        v.push_back(position[3*i + 1]);
        v.push_back(position[3*i + 2]);

        v.push_back(texcoord[2*i + 0]);
        v.push_back(texcoord[2*i + 1]);
    }

    Buffer* newBuffer = new Buffer(v);
    vector<int> newindices;
    int indexCount = newindices.size();

    newBuffer->name = name + "Buffer";

    for( int i = 0; i < indexCount; i++ )
        newindices.push_back(indices[i]);

    IndexBuffer* newIndexBuffer = new IndexBuffer(indices);

    newIndexBuffer->name = name + "IndexBuffer";

    Geometry* newGeometry = new Geometry;

    newGeometry->name = name + "Geometry";

    (*newGeometry)["position"] = Field(newBuffer, 3,5,0);
    (*newGeometry)["texcoord"] = Field(newBuffer, 2,5,3);
    newGeometry->indices = newIndexBuffer;

    FatteningGeometryInfo info;

    info.geometry = newGeometry;
    info.buffer = newBuffer;
    info.indexBuffer = newIndexBuffer;

    return info;
}

void Fattening::populateModel(Model* model, Shape* shape)
{
    FatteningGeometryInfo info = newGeometry(shape->name);

    model->add(info.geometry, true);
    model->add(info.buffer, true);
    model->add(info.indexBuffer, true);

    shape->geometry = info.geometry;
}

} // end namespace

