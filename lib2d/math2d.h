#ifndef _MATH2D_
#define _MATH2D_


#include <stdio.h>
#include <math.h>
#include <assert.h>

#include <vector>
#include <list>
#include <map>
#include <set>

#include "lin/lin.h"

// TRIG_TABLE_SIZE must be a power of 2
#ifndef TRIG_TABLE_SIZE
#define TRIG_TABLE_SIZE 65536
#endif

using namespace std;

namespace lib2d
{

#define sgn(X) ((X)<0?(-1):1)
#define abs(X) ((X)<0?(-(X)):(X))
int mymod( int n, int m );
double squareOf(double t);
double min( double n, double m );
double max( double n, double m );

#define kBigPrime 100003
#define PI (3.1415926535897931)
#define ONE_OVER_PI (0.31830988618379069)

#define kRFactor 0.75 //should be bigger than sqrt(2)/2


void buildTrigTable();

struct trig_pair {double cos,sin;};
typedef trig_pair (*trig_function) (double);
trig_pair factory_sin_and_cos(double);
trig_pair lut_sin_and_cos(double);

trig_pair fsin_and_cos(double theta); //wrapper for sin_and_cos


vector<double> quad_form(double a, double b, double c);

#define QUAD_FORM(A,B,C,r1,r2) { double a=A,b=B,c=C; double q = -0.5*( b + sgn(b)*sqrt(b*b - 4.0*a*c) ); r1 = q/a; r2 = c/q; }

pair<double,double> findShortestEncapsulatingInterval( vector<double>& L, double perimeter );

class OverlapInfo
{
public:
    Vec2 normal, contactPoint;
    
    OverlapInfo()
    {
        tare();
    }
    
    void tare()
    {
        normal.set(0,0);
        contactPoint.set(0,0);
    }
    
    void print()
    {
        printf("normal = "); normal.print(); printf( "    " );
        printf("contact point = "); contactPoint.print();
    }
    
    void display()
    {
        print();
        printf("\n");
    }
};

typedef enum shape_type {
    kCircle  = 0x01,
    kPolygon = 0x02,
    kBase = 0x10,
    kCircleThenCircle   = 0x11,
    kCircleThenPolygon  = 0x12,
    kPolygonThenCircle  = 0x21,
    kPolygonThenPolygon = 0x22
} shape_type;


class Shape {
public:
    shape_type tag;
    virtual ~Shape() {}
    virtual Vec2 pointIn() {return Vec2();}
    virtual bool overlaps( class Shape& S, OverlapInfo* oInfo = NULL )
        {assert(false); return true;}
};

shape_type shape_then(const class Shape* A, const class Shape* B);
shape_type shape_then(const class Shape& A, const class Shape& B);



class Polygon : public Shape
{
public: // (just for this program though)
    vector<Vec2> L;
	
public:
    Polygon() { tag=kPolygon; }
    ~Polygon() {}
    
    class Polygon& add( const Vec2& V );
    class Polygon& add( double x, double y );
    class Polygon& operator += (const Vec2& V);
    class Polygon& operator -= (const Vec2& V);
	class Polygon operator* (double k) const;
	class Polygon operator/ (double k) const;
	inline friend class Polygon operator * (double k, class Polygon P) {return P*k;}
	
    void display() const;
    void print() const;
    Vec2 pointIn() const;
    void rotate(double theta);
    
    void draw() {
#if GL_ON
    
    vector<Polygon> pl = triangulate();
    
    for( vector<Polygon>::iterator itr = pl.begin(); itr!=pl.end(); itr++ )
    {
        glColor3f(1,1,1);
        glBegin(GL_POLYGON);
        
        const vector<Vec2>& tl = itr->points();
        for( vector<Vec2>::const_iterator itr = tl.begin(); itr!=tl.end(); itr++ )
            glVertex3f( (int)(itr->x), (int)(itr->y), 1 );
        
        glEnd();
    }
#endif
    }
    
    Vec2 average() const;
    
    double perimeter() const;
    double area() const;
    Vec2 centroid() const;
    Vec2 shortestPathOut(const Vec2& P) const;
    const vector<Vec2>& points() const {return L;}
    bool vectorInside( const Vec2& V ) const;
    bool overlaps( const class Polygon& P, OverlapInfo* oInfo = NULL ) const;
    bool overlaps( const class Circle& C, OverlapInfo* oInfo = NULL ) const;
    vector<class Polygon> triangulate() const;
    void continuousOverlapInfo( int a, double ta, int b, double tb, class OverlapInfo* cInfo ) const;
    void continuousOverlapInfo( vector<double>& L, double perimeter, OverlapInfo* cInfo ) const;
    int windingNumber() const;
    void reverse();
    
    bool simple() const;
    
    pair<int,double> sideOfPosition(double t) const;
};


class Circle : public Shape
{
public:
    Vec2 C;
    double r;
    
    Circle() {
        tag = kCircle;
        C = Vec2();
        r = 0;
    }
    
    Circle( const Vec2& inc, double inr ) {
        tag = kCircle;
        r = inr; C = inc;
    }
    
    Circle( double inx, double iny, double inr ) {
        tag = kCircle;
        r = inr; C = Vec2(inx,iny);
    }
    
    void set( const Vec2& inc, double inr );
    void set( double inx, double iny, double inr );
    bool vectorInside( const Vec2 V ) const;
    class Circle& operator -= (const Vec2& V);
    class Circle& operator += (const Vec2& V);
	class Circle operator* (double k) const;
	class Circle operator/ (double k) const;
	inline friend class Circle operator * (double k, class Circle inC) {return inC*k;}
	
    void print() const;
    void display() const;
    
    void draw() {
#if GL_ON
        const int FACETS = 32;
        glBegin(GL_POLYGON);
        glColor3f(1,1,1);
        for( int i=0; i<FACETS; i++ )
        {
            glVertex3f( C.x+r*cos(2.0*PI*i/FACETS), C.y+r*sin(2.0*PI*i/FACETS), 1 );
        }
        glEnd();
#endif
    }
    
    Vec2 pointIn() const;
    
    const vector<double> intersectLine( const Vec2& P, const Vec2& Q ) const;
    bool overlaps( const class Polygon& P, OverlapInfo* oInfo = NULL ) const;
    bool overlaps( const class Circle& inC, OverlapInfo* oInfo = NULL ) const;
    void continuousOverlapInfo( const class Circle& inC, OverlapInfo* oInfo ) const;
    
    void rotate(double theta);    
};


} //end namespace

#endif




