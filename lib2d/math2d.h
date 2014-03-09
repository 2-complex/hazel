#ifndef _MATH2D_
#define _MATH2D_


#include <stdio.h>
#include <math.h>
#include <assert.h>

#include <vector>
#include <list>
#include <map>
#include <set>


#ifndef GL_ON
#define GL_ON 1
#endif

// TRIG_TABLE_SIZE must be a power of 2
#ifndef TRIG_TABLE_SIZE
#define TRIG_TABLE_SIZE 65536
#endif

#if GL_ON
#include <glut/glut.h>
#endif


using namespace std;

namespace lib2d {


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



class Vector2d {
public:
    
    double x,y;
    
    Vector2d() { x=y=0; }
    Vector2d( double inx, double iny ) { x = inx; y = iny; }
    Vector2d( const class Vector2d& V ) { x = V.x; y = V.y; }
    ~Vector2d() {}
    
    inline double mag() const { return (sqrt(x*x+y*y)); }
    inline double magSquared() const { return (x*x+y*y); }
    inline double dot( const class Vector2d& V ) const { return ( x*V.x + y*V.y ); }
    inline void set( double inx, double iny ) {x = inx; y = iny;}
    
    class Vector2d projectOnto( const class Vector2d& V ) const;
    class Vector2d reflectAbout( const class Vector2d& V ) const;
    
    Vector2d perp() const;
    Vector2d right() const;
    Vector2d left() const;
    double angle() const;
    
    double theta( const Vector2d V ) const;
    double angleTo( const Vector2d V ) const;
    
    bool between(const Vector2d U, const Vector2d V) const;
    bool betweenInclusive(const Vector2d U, const Vector2d V) const;
    
    
    inline class Vector2d& operator = (const class Vector2d& V) {
        x = V.x;
        y = V.y;
        return *this;
    }
    
    inline class Vector2d operator - () const {
        Vector2d V( -x,-y );
        return V;
    }
    
    inline friend class Vector2d operator * (Vector2d V, double r) {
        Vector2d U;
        U.x = V.x*r;
        U.y = V.y*r;
        return U;
    }
    
    inline friend class Vector2d operator * (double r, Vector2d V) {
        return V*r;
    }
    
    inline class Vector2d operator / (double r) const {
        Vector2d V;
        V.x = x/r;
        V.y = y/r;
        return V;
    }
    
    inline class Vector2d operator + (const class Vector2d& V) const {
        Vector2d U;
        U.x = x+V.x;
        U.y = y+V.y;
        return U;
    }
    
    inline class Vector2d operator - (const class Vector2d& V) const {
        Vector2d U;
        U.x = x-V.x;
        U.y = y-V.y;
        return U;
    }
    
    inline class Vector2d& operator += (const class Vector2d& V) {
        x+=V.x; y+=V.y;
        return *this;
    }
    
    inline class Vector2d& operator -= (const class Vector2d& V) {
        x-=V.x; y-=V.y;
        return *this;
    }
    
    inline class Vector2d& operator *= (double r) {
        x*=r; y*=r;
        return *this;
    }
    
    inline class Vector2d& operator /= (double r) {
        x/=r; y/=r;
        return *this;
    }
    
    inline bool operator == (const Vector2d& V) const {
        return (x==V.x && y==V.y);
    }
    
    inline bool operator != (const Vector2d& V) const {
        return !(x==V.x && y==V.y);
    }

  
    class Vector2d& rotate(double theta);
    class Vector2d& rotate(const trig_pair p);
    
    void display() const;
    void print() const;

    static const Vector2d randVector( double amount = 1.0 ) {
        double t = (double)(random());
        double u = (double)(random()%1000001)/1000000.0;
        return Vector2d( cos(t), sin(t) ) * amount * u;
    }
};

Vector2d normalize( const Vector2d& V );


class Vector3d {
public:
    double x,y,z;
 
    Vector3d() {x=y=z=0;}
    Vector3d( double inx, double iny, double inz ) { x = inx; y = iny; z = inz; }
    Vector3d( const Vector2d& V ) { x = V.x; y = V.y; z = 0.0; }
    ~Vector3d() {}
    
    
    inline void set( double inx, double iny, double inz ) {
        x = inx; y = iny; z = inz;
    }
    
    inline double mag() const {
        return (sqrt(x*x+y*y+z*z));
    }
    
    inline double magSquared() const {
        return (x*x + y*y + z*z);
    }
    
    inline double dot( const class Vector3d& V ) const {
        return ( x*V.x + y*V.y + z*V.z );
    }
    
    inline class Vector3d cross( const class Vector3d& V ) const {
        Vector3d retval( 	y*V.z - z*V.y,
                            z*V.x - x*V.z,
                            x*V.y - y*V.x );
        return retval;
    }

    
    class Vector3d projectOnto( const class Vector3d& V ) const;
    class Vector3d reflectAbout( const class Vector3d& V ) const;
    double theta( const Vector3d V ) const;
    
    
    Vector2d& operator = (Vector2d& V) {
        x = V.x; y = V.y; z = 0.0;
		return V;
    }
        
    inline class Vector3d operator - () const{
        Vector3d V( -x,-y,-z );
        return V;
    }
    
    inline friend class Vector3d operator * (Vector3d V, double r) {
        Vector3d U;
        U.x = V.x*r;
        U.y = V.y*r;
        U.z = V.z*r;
        return U;
    }
    
    inline friend class Vector3d operator * (double r, Vector3d V) {
        return V*r;
    }
    
    inline class Vector3d operator / (double r) const {
        Vector3d V(x/r, y/r, z/r);
        return V;
    }
    
    inline class Vector3d operator + (const class Vector3d& V) const {
        Vector3d U;
        U.x = x+V.x;
        U.y = y+V.y;
        U.z = z+V.z;
        return U;
    }
    
    inline class Vector3d operator - (const class Vector3d& V) const {
        Vector3d U;
        U.x = x-V.x;
        U.y = y-V.y;
        U.z = z-V.z;
        return U;
    }
    
    inline class Vector3d& operator += (const class Vector3d& V) {
        x+=V.x; y+=V.y; z+=V.y;
        return *this;
    }
    
    inline class Vector3d& operator -= (const class Vector3d& V) {
        x-=V.x; y-=V.y; z-=V.z;
        return *this;
    }
    
    inline class Vector3d& operator *= (double r) {
        x*=r; y*=r; z*=r;
        return *this;
    }
    
    inline class Vector3d& operator /= (double r) {
        x/=r; y/=r; z/=r;
        return *this;
    }
    
    inline bool operator == (const Vector3d& V) const {
        return (x==V.x && y==V.y && z==V.z);
    }
    
    inline bool operator != (const Vector3d& V) const {
        return !(x==V.x && y==V.y && z==V.z);
    }
    
    static const Vector3d randVector( double amount = 1.0 ) {
        double t = (double)(random());
        double u = (double)(random());
        double v = (double)(random()%1000001)/1000000.0;
        return Vector3d( sin(u)*cos(t), sin(u)*sin(t), cos(u) ) * amount * v;
    }

    
    void display() const;
    void print() const;
};


Vector3d normalize( const Vector3d& V );



/*An ad hoc matrix class*/

class Matrix3d
{
public:
    float E[3][3];
    
    Matrix3d()
    {
        E[0][0]=1; E[1][0]=0; E[2][0]=0;
        E[0][1]=0; E[1][1]=1; E[2][1]=0;
        E[0][2]=0; E[1][2]=0; E[2][2]=1;
    }
    
    Matrix3d(float a, float b, float c,
             float d, float e, float f,
             float g, float h, float i )
    {
        E[0][0]=a; E[1][0]=b; E[2][0]=c;
        E[0][1]=d; E[1][1]=e; E[2][1]=f;
        E[0][2]=g; E[1][2]=h; E[2][2]=i;
    }
    
    Matrix3d(Vector3d& u, Vector3d& v, Vector3d& w)
    {
        E[0][0]=u.x; E[1][0]=v.x; E[2][0]=w.x;
        E[0][1]=u.y; E[1][1]=v.y; E[2][1]=w.y;
        E[0][2]=u.z; E[1][2]=v.z; E[2][2]=w.z;
    }
    
    Matrix3d& set(Vector3d& u, Vector3d& v, Vector3d& w)
    {
        E[0][0]=u.x; E[1][0]=v.x; E[2][0]=w.x;
        E[0][1]=u.y; E[1][1]=v.y; E[2][1]=w.y;
        E[0][2]=u.z; E[1][2]=v.z; E[2][2]=w.z;
		return *this;
    }
    
    Matrix3d& set(  float a, float b, float c,
                    float d, float e, float f,
                    float g, float h, float i  )
    {
        E[0][0]=a;E[1][0]=b;E[2][0]=c;
        E[0][1]=d;E[1][1]=e;E[2][1]=f;
        E[0][2]=g;E[1][2]=h;E[2][2]=i;
        return *this;
    }
    
    ~Matrix3d(){};
    
    
    const bool operator ==(const Matrix3d& M) const
    {
        for(long x=0; x<3; x++)
        for(long y=0; y<3; y++)
        {
            if( M.E[x][y]!=E[x][y] )
                return false;
        }
        return true;
    }
    
    
    class Matrix3d& operator +=(const Matrix3d& M)
    {
        for(long x=0; x<3; x++)
        for(long y=0; y<3; y++)
        {
            E[x][y] += M.E[x][y];
        }
        return *this;
    }
    
    class Matrix3d& operator -=(const Matrix3d& M)
    {
        for(long x=0; x<3; x++)
        for(long y=0; y<3; y++)
        {
            E[x][y] -= M.E[x][y];
        }
        return *this;
    }
    
    
    class Matrix3d operator +(const Matrix3d& M) const
    {
        Matrix3d R;
        for(long x=0; x<3; x++)
        for(long y=0; y<3; y++)
            R.E[x][y] = E[x][y] + M.E[x][y];
        
        return R;
    }
    
    class Matrix3d operator -(const Matrix3d& M) const
    {
        Matrix3d R;
        for(long x=0; x<3; x++)
        for(long y=0; y<3; y++)
            R.E[x][y] = E[x][y] - M.E[x][y];
        
        return R;
    }
    
    class Matrix3d operator *=(double k)
    {
        Matrix3d R;
        for(long x=0; x<3; x++)
        for(long y=0; y<3; y++)
            E[x][y] *= k;
        
        return R;
    }
    
    Matrix3d& operator /=(double k)
    {
        for(long x=0; x<3; x++)
        for(long y=0; y<3; y++)
            E[x][y] /= k;
        
        return *this;
    }
    
    
    friend class Matrix3d operator * (Matrix3d T, double k)
    {
        Matrix3d R;
        for(long x=0; x<3; x++)
        for(long y=0; y<3; y++)
            R.E[x][y] = T.E[x][y]*k;
        
        return R;    
    }
    
    friend class Matrix3d operator * (double k, Matrix3d V)
    {
        return V*k;
    }
    
    
    class Matrix3d operator /(double k) const
    {
        Matrix3d R;
        for(long x=0; x<3; x++)
        for(long y=0; y<3; y++)
            R.E[x][y] = E[x][y]/k;
        
        return R;
    }

    
    Vector3d row(int i) const
    {
        return Vector3d( E[0][i], E[1][i], E[2][i] );
    }
    
    Vector3d column(int i) const
    {
        return Vector3d( E[i][0], E[i][1], E[i][2] );
    }
    
    Vector3d operator *(const Vector3d& V) const
    {
        return Vector3d(   E[0][0]*V.x + E[1][0]*V.y + E[2][0]*V.z,
                           E[0][1]*V.x + E[1][1]*V.y + E[2][1]*V.z,
                           E[0][2]*V.x + E[1][2]*V.y + E[2][2]*V.z );
    }
                    
    Matrix3d& operator *=( Matrix3d& M )
    {
        Matrix3d B;
        for(long x=0; x<3; x++)
        for(long y=0; y<3; y++)
        {
            B.E[x][y] = ( row(y) ).dot( M.column(x) );
        }
        (*this) = B;
        return *this;
    }
    
    class Matrix3d operator *( const Matrix3d& M ) const
    {
        Matrix3d B;
        for(long x=0; x<3; x++)
        for(long y=0; y<3; y++)
        {
            B.E[x][y] = ( row(y) ).dot( M.column(x) );
        }
        return B;
    }
    
    
    float det() const
    {
        return(  E[0][0]*E[1][1]*E[2][2]
                -E[0][0]*E[1][2]*E[2][1]
                +E[0][1]*E[1][2]*E[2][0]
                -E[0][1]*E[1][0]*E[2][2]
                +E[0][2]*E[1][0]*E[2][1]
                -E[0][2]*E[1][1]*E[2][0] );
    }
    
    
    void checkerNegate()
    {
        E[1][0] = -E[1][0];
        E[0][1] = -E[0][1];
        E[2][1] = -E[2][1];
        E[1][2] = -E[1][2];
    }
    
    class Matrix3d transpose() const
    {
        Matrix3d B;
        for(long x=0; x<3; x++)
        for(long y=0; y<3; y++)
        {
            B.E[x][y] = E[y][x];
        }
        return B;
    }
    
    
    double codet( long x,long y ) const
    {
        static Matrix3d B;
        B = (*this);
        
        B.E[x][0] = B.E[x][1] = B.E[x][2] = 0;
        B.E[0][y] = B.E[1][y] = B.E[2][y] = 0;
        B.E[x][y] = 1;
        
        return B.det();
    }
    
                                                                                
    class Matrix3d inverse() const
    {
        Matrix3d B;
        
        for(long x=0; x<3; x++)
        for(long y=0; y<3; y++)
            B.E[x][y] = codet(y,x);
        
        B/=det();
        
        return B;
    }
		
		
    void print() const
    {
        printf( "%f, %f, %f\n",E[0][0],E[1][0],E[2][0] );
        printf( "%f, %f, %f\n",E[0][1],E[1][1],E[2][1] );
        printf( "%f, %f, %f\n",E[0][2],E[1][2],E[2][2] );
    }
    
    void display() const
    {
        print(); printf( "\n" );
    }
};




class Line2d {
public:
    Vector2d P;
    Vector2d Q;
    
    Line2d( double inx1, double iny1, double inx2, double iny2 ) {
        P.x = inx1;
        P.y = iny1;
        Q.x = inx2;
        Q.y = iny2;
    }
    
    Line2d( const Vector2d& inP, const Vector2d& inQ ) { P = inP; Q = inQ; }
    
    const vector<double> intersectLine( const class Line2d& L ) const;
};


template <class _pclass, class _qclass>
void pushOut(_pclass& P, _qclass& Q, Vector2d P_v, Vector2d Q_v)
{
    assert(Q_v-P_v != Vector2d(0,0));
    
    int i=0;
    while( P.overlaps(Q) )
    {
        Q+=Q_v; P+=P_v;
        P_v*=2.0; Q_v*=2.0;
        i++;
    }
    P_v*=0.5; Q_v*=0.5;
    while(i--)
    {
        Q-=Q_v; P-=P_v;
        if( P.overlaps(Q) ){ Q+=Q_v; P+=P_v; }
        P_v*=0.5; Q_v*=0.5;
    }
}

pair<double,double> findShortestEncapsulatingInterval( vector<double>& L, double perimeter );

class OverlapInfo
{
public:
    Vector2d normal, contactPoint;
    
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
    virtual Vector2d pointIn() {return Vector2d();}
    virtual bool overlaps( class Shape& S, OverlapInfo* oInfo = NULL )
        {assert(false); return true;}
};

shape_type shape_then(const class Shape* A, const class Shape* B);
shape_type shape_then(const class Shape& A, const class Shape& B);



class Polygon : public Shape {
public: // (just for this program though)
    vector<Vector2d> L;
	
public:
    Polygon() { tag=kPolygon; }
    ~Polygon() {}
    
    class Polygon& add( const Vector2d& V );
    class Polygon& add( double x, double y );
    class Polygon& operator += (const Vector2d& V);
    class Polygon& operator -= (const Vector2d& V);
	class Polygon operator* (double k) const;
	class Polygon operator/ (double k) const;
	inline friend class Polygon operator * (double k, class Polygon P) {return P*k;}
	
    void display() const;
    void print() const;
    Vector2d pointIn() const;
    void rotate(double theta);
    
    void draw() {
#if GL_ON
    
    vector<Polygon> pl = triangulate();
    
    for( vector<Polygon>::iterator itr = pl.begin(); itr!=pl.end(); itr++ )
    {
        glColor3f(1,1,1);
        glBegin(GL_POLYGON);
        
        const vector<Vector2d>& tl = itr->points();
        for( vector<Vector2d>::const_iterator itr = tl.begin(); itr!=tl.end(); itr++ )
            glVertex3f( (int)(itr->x), (int)(itr->y), 1 );
        
        glEnd();
    }
#endif
    }
    
    Vector2d average() const;
    
    double perimeter() const;
    double area() const;
    Vector2d centroid() const;
    Vector2d shortestPathOut(const Vector2d& P) const;
    const vector<Vector2d>& points() const {return L;}
    bool vectorInside( const Vector2d& V ) const;
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
    Vector2d C;
    double r;
    
    Circle() {
        tag = kCircle;
        C = Vector2d();
        r = 0;
    }
    
    Circle( const Vector2d& inc, double inr ) {
        tag = kCircle;
        r = inr; C = inc;
    }
    
    Circle( double inx, double iny, double inr ) {
        tag = kCircle;
        r = inr; C = Vector2d(inx,iny);
    }
    
    void set( const Vector2d& inc, double inr );
    void set( double inx, double iny, double inr );
    bool vectorInside( const Vector2d V ) const;
    class Circle& operator -= (const Vector2d& V);
    class Circle& operator += (const Vector2d& V);
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
    
    Vector2d pointIn() const;
    
    const vector<double> intersectLine( const Vector2d& P, const Vector2d& Q ) const;
    bool overlaps( const class Polygon& P, OverlapInfo* oInfo = NULL ) const;
    bool overlaps( const class Circle& inC, OverlapInfo* oInfo = NULL ) const;
    void continuousOverlapInfo( const class Circle& inC, OverlapInfo* oInfo ) const;
    
    void rotate(double theta);    
};


} //end namespace

#endif




