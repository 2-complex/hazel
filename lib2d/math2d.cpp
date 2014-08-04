
#include <algorithm>

#include "math2d.h"

#if GL_ON
#include "opengl.h"
#endif

namespace lib2d {

int mymod( int n, int m ) { return ((n%m)+m)%m; }
double squareOf(double t){ return t*t; }
double min( double n, double m ) {return n<m?n:m;}
double max( double n, double m ) {return n<m?m:n;}

Vec2 normalize( const Vec2& V ) {return V/V.mag();}
Vec3 normalize( const Vec3& V ) {return V/V.mag();}


double* sin_table = NULL;
trig_function sin_and_cos = factory_sin_and_cos;

void buildTrigTable()
{
    if( !sin_table )
        sin_table = (double*)malloc((sizeof(double))*TRIG_TABLE_SIZE); // never freed, really
    for( int i =0; i<TRIG_TABLE_SIZE; i++ )
        sin_table[i] = sin(2.0*PI*i/TRIG_TABLE_SIZE);
    sin_and_cos = lut_sin_and_cos;
}

trig_pair factory_sin_and_cos(double theta)
{
    trig_pair p;
    p.sin = sin(theta);
    p.cos = cos(theta);
    return p;
}

trig_pair lut_sin_and_cos(double theta)
{
    trig_pair p;
    
    int t = (int)((0.5)*theta*TRIG_TABLE_SIZE*ONE_OVER_PI + 0.5);
    int u = TRIG_TABLE_SIZE/4 - t;
    int mask = ((TRIG_TABLE_SIZE)-1);
    u = u & mask;
    t = t & mask;
    
    p.sin = sin_table[t];
    p.cos = sin_table[u];
    
    return p;
}

trig_pair fsin_and_cos(double theta)
{
    return sin_and_cos(theta);
}


vector<double> quad_form(double a, double b, double c)
{
    vector<double> R(2);
  
    double q = -0.5*( b + sgn(b)*sqrt(b*b - 4.0*a*c) );
    R[0] = q/a;
    R[1] = c/q;
  
    return R;
}

Vec2 projectOnto( const Vec2& T, const Vec2& V )
{
    return V*T.dot(V)/V.magSquared();
}

Vec2 reflectAbout( const Vec2& T, const Vec2& V )
{
    return 2.0 * projectOnto(T, V) - T;
}

Vec2 right( const Vec2& T )
{
    Vec2 V;
    V.x = T.y;
    V.y = -T.x;
    return V;
}

Vec2 left( const Vec2& T )
{
    return -right(T);
}

double angle( const Vec2& T )
{
    return atan2(T.y, T.x);
}

double theta( const Vec2& T, const Vec2& V )
{
    return acos(T.dot(V)/(T.mag()*V.mag()));
}

double angleTo( const Vec2& T, const Vec2& V )
{
    return theta(T, V)*sgn( V.y*T.x-V.x*T.y );
}

bool between( const Vec2& T, const Vec2& U, const Vec2& V )
{
    return V.y*T.x -V.x*T.y > 0 &&  U.x*T.y -U.y*T.x > 0;
}

bool betweenInclusive( const Vec2& T, const Vec2 U, const Vec2 V )
{
    return V.y*T.x -V.x*T.y >= 0 &&  U.x*T.y - U.y*T.x >= 0;
}

Vec2& rotate( Vec2& T, const trig_pair& p )
{
    double new_x, new_y;
    new_x = p.cos*T.x - p.sin*T.y;
    new_y = p.sin*T.x + p.cos*T.y;
    T.x = new_x;
    T.y = new_y;
    return T;
}

Vec2& rotate( Vec2& T, double theta )
{
    trig_pair p = sin_and_cos(theta);
    return rotate(T, p);
}

Vec3 projectOnto( const Vec3& T, const Vec3& V )
{
    return ( V*T.dot(V)/V.magSquared() );
}

Vec3 reflectAbout( const Vec3& T, const class Vec3& V )
{
    return 2.0*projectOnto(T, V) - T;
}

double theta( const Vec3& T, const Vec3& V )
{
    return acos(T.dot(V)/(T.mag()*V.mag()));
}


Line2d::Line2d(){}

Line2d::Line2d(const Vec2& P, const Vec2& Q)
    : P(P), Q(Q)
{
}

const vector<double> Line2d::intersectLine( const class Line2d& L ) const
{
    vector<double> R(2);
    double det = (Q.x-P.x)*(L.P.y-L.Q.y) - (Q.y-P.y)*(L.P.x-L.Q.x);
    R[0] = (L.P.y-L.Q.y)*(L.P.x-P.x) - (L.P.x-L.Q.x)*(L.P.y-P.y);
    R[1] = (Q.x-P.x)*(L.P.y-P.y)     - (Q.y-P.y)*(L.P.x-P.x);
    R[0] /= det;
    R[1] /= det;
    return R;
}



class Polygon& Polygon::add( const Vec2& V )
{	
    L.push_back(V);
    return *this;
}

class Polygon& Polygon::add( double x, double y )
{
    L.push_back(Vec2(x,y));
    return *this;
}


class Polygon& Polygon::operator += (const Vec2& V) 
{
    vector<Vec2>::iterator itr;
    for( itr=L.begin(); itr!=L.end(); itr++ )
        *itr+=V;
    return *this;
}

class Polygon& Polygon::operator -= (const Vec2& V)
{
    vector<Vec2>::iterator itr;
    for( itr=L.begin(); itr!=L.end(); itr++ )
        *itr-=V;
    return *this;
}

class Polygon Polygon::operator* (double k) const
{
	Polygon P(*this);
	for( vector<Vec2>::iterator itr=P.L.begin(); itr!=P.L.end(); itr++ )
        *itr*=k;
	return P;
}

class Polygon Polygon::operator/ (double k) const
{
	Polygon P(*this);
	for( vector<Vec2>::iterator itr=P.L.begin(); itr!=P.L.end(); itr++ )
        *itr/=k;
	return P;
}

void Polygon::display() const
{
    vector<Vec2>::const_iterator itr;
    for( itr=L.begin(); itr!=L.end(); itr++ )
        itr->display();
}

void Polygon::print() const
{
    vector<Vec2>::const_iterator itr;
    for( itr=L.begin(); itr!=L.end(); itr++ )
    {
        itr->print();
        printf( ", " );
    }
}

Vec2 Polygon::pointIn() const
{
    return L[0];
}

void Polygon::rotate(double theta)
{
    trig_pair p = sin_and_cos(theta);
    for( vector<Vec2>::iterator itr = L.begin(); itr!= L.end(); itr++ )
        lib2d::rotate(*itr, p);
}


void Polygon::draw() const
{
#if GL_ON
    glColor3f(1,0,0);
    glBegin(GL_LINE_LOOP);

    const vector<Vec2>& tl = points();
    for( vector<Vec2>::const_iterator itr = tl.begin(); itr!=tl.end(); itr++ )
    {
        glVertex3f( itr->x, itr->y, 1.0 );
    }

    glEnd();
#endif
}

bool Polygon::vectorInside( const Vec2& V ) const 
{    
    double theta = 0;
    int size = L.size();
    for( int i=0; i<size; i++ )
        theta += angleTo(L[i]-V, L[(i+1)%size]-V);
    return ( theta > 1 || theta < -1 );
}

Vec2 Polygon::average() const
{
    Vec2 R;
    int size = L.size();
    for( int i=0; i<size; i++ )
        R+=L[i];
    return R/((double)size);
}


double Polygon::perimeter() const
{
    double accum=0.0;
    int n = L.size();
    for(int i=0; i<n; i++)
    {
        int j=(i+1)%n;
        accum+=(L[j]-L[i]).mag();
    }
    return accum;
}

double Polygon::area() const
{
    double accum=0.0;
    int n = L.size();
    for(int i=0; i<n; i++)
    {
        int j=(i+1)%n;
        accum+=L[j].y*L[i].x - L[i].y*L[j].x;
    }
    return 0.5*accum;
}

Vec2 Polygon::centroid() const
{
    double accumx=0.0, accumy=0.0;
    int n = L.size();
    for(int i=0; i<n; i++)
    {
        int j=(i+1)%n;
        double t=L[i].x*L[j].y - L[j].x*L[i].y;
        accumx+= (L[i].x+L[j].x)*t;
        accumy+= (L[i].y+L[j].y)*t;
    }
    return Vec2(accumx, accumy)/(6.0*area());
}

void Polygon::reverse()
{
    int size = L.size();
    vector<Vec2> newL(size);
    for( int i=0; i<size; i++ )
        newL[i] = L[size-i-1];
    L = newL;
}


bool Polygon::overlaps( const class Polygon& P, OverlapInfo* oInfo ) const
{
    vector<Vec2>::const_iterator itr;
    bool retval = false;
    
    if( !oInfo )
    {
        // is there a point of this inside the polygon P?
        for( itr=L.begin(); itr!=L.end(); itr++ )
            if( P.vectorInside( *itr ) )
                return true;
    
        // is there a point of the polygon P inside 'this' polygon?
        for( itr=P.L.begin(); itr!=P.L.end(); itr++ )
            if( vectorInside( *itr ) )
                return true;
    }
    
    vector<double> TtouchPositions, PtouchPositions;
    double TlengthAccum=0.0, PlengthAccum=0.0, TsegmentLength=0.0, PsegmentLength=0.0;
    
    
    // Do the lines of one polygon intersect the lines of the other anywhere?
    int lsize = L.size(), plsize = P.L.size();
    for( int j=0; j<plsize; j++, PlengthAccum+=PsegmentLength )
    {
        TsegmentLength = TlengthAccum = 0.0;
        
        for( int i=0; i<lsize; i++, TlengthAccum+=TsegmentLength )
        {
            Vec2    T0 = L[i], T1 = L[(i+1)%lsize],
                        PT0 = P.L[j], PT1 = P.L[(j+1)%plsize];
            TsegmentLength = (T1-T0).mag();
            PsegmentLength = (PT1-PT0).mag();
            
            vector<double> R = Line2d(T0,T1).intersectLine(Line2d(PT0,PT1));
            if( R[0]>=0.0 && R[0]<=1.0 && R[1]>=0.0 && R[1]<=1.0 )
            {
                if( !oInfo ) return true;
                
                TtouchPositions.push_back( TlengthAccum + TsegmentLength*R[0] );
                PtouchPositions.push_back( PlengthAccum + PsegmentLength*R[1] );
                
                retval = true;
            }
        }
    }
    
    if(retval && oInfo)
        continuousOverlapInfo(TtouchPositions, perimeter(), oInfo);
    
    return retval;
}




Vec2 Polygon::shortestPathOut(const Vec2& P) const
{
    vector<Vec2>::const_iterator itr;
    Vec2 V(L[0] - P);
    
    //first take the minimum distance to a vertex
    for( itr=L.begin(); itr!=L.end(); itr++ )
        if( V.magSquared()>((*itr)-P).magSquared() ) V = (*itr)-P;
    
    //then take the minimum distance to an edge IF that actually excapes.
    int size = L.size();
    for( int i=0; i<size; i++ )
    {
        Vec2 A(P-L[i]);
        Vec2 B(L[(i+1)%size]-L[i]);
        
        double d = A.dot(B);
        if( d >= 0 && d <= B.magSquared() )
        {
            Vec2 C = projectOnto(A, B)-A;
            if( V.magSquared() > C.magSquared() )
            {
                //accuracy issues in the situation when the point 
                //is extremely close to the line suggested to me that I should do this:
                V = right(B);
                V *= C.mag()/V.mag();
            }
        }
    }
    
    return V;
}




bool Polygon::overlaps( const class Circle& C, OverlapInfo* oInfo ) const
{
    bool retval=false;
    
    vector<double> touchPositions;
    double lengthAccum=0;
    
    int size = L.size();
    
    if( !oInfo )
    {
        for( int i=0; i<size; i++ )
        {
            //if the circle is nearer than r to the ith vertex
            if( C.r*C.r - (L[i]-C.C).magSquared() > 0.0 )
                return true;
        }
    }
    
    double segmentLength=0.0;
    for( int i=0; i<size; i++, lengthAccum += segmentLength )
    {
        Vec2 P(L[i]),Q(L[(i+1)%size]);
        segmentLength = (P-Q).mag();
        
        //bounding box check
        Vec2 A = P-C.C, B = Q-C.C;
        if ((A.x > C.r && B.x > C.r) || (A.y > C.r && B.y > C.r) ||
            (A.x < -C.r && B.x < -C.r) || (A.y < -C.r && B.y < -C.r)) continue;
        //end of bounding box check
        
        vector<double> R = C.intersectLine(L[i],L[(i+1)%size]);
        
        bool R0in(R[0]>=0 && R[0]<=1), R1in(R[1]>=0 && R[1]<=1);
        if( R0in || R1in )
        {
            if( !oInfo ) return true;
            
            if(R0in)
                touchPositions.push_back( lengthAccum + segmentLength*R[0] );
            if(R1in)
                touchPositions.push_back( lengthAccum + segmentLength*R[1] );
            
            retval = true;
        }
    }
    
    if(retval)
    {
        if(oInfo)
            continuousOverlapInfo(touchPositions, perimeter(), oInfo);
    }
    else
    {
        if( vectorInside(C.C) )
        {
            if( oInfo )
            {
                oInfo->normal = shortestPathOut(C.C);
                oInfo->contactPoint = C.C - C.r*normalize(oInfo->normal);
                
                retval = true;
            }
            else
                return true;
        }
    }
    
    return retval;
}


pair<double,double> findShortestEncapsulatingInterval( vector<double>& L, double perimeter )
{
    pair<double,double> R;
    
    double best_w = perimeter+1.0;
    sort(L.begin(), L.end());
    
    int n = L.size();
    
    /*first check the first and last and see if they work*/
    double  a = L[0],
            b = L[n-1];
    
    best_w = b-a;
    R.first = a;
    R.second = b;
    if( best_w<0.5*perimeter )
        return R;
    
    /*if there isn't an obvious right answer, exhaust the possibilities*/
    for( int i=0; i<n; i++ )
    {
        int z = ((i+1)/n),
            j = (i+1)%n; //z is for a trick for handling the last element of L
        a = L[i];
        b = L[j];
        double w = z*perimeter+b-a;
        w = perimeter - w;
        
        if( w<best_w )
        {
            R.first = b; //experiment
            R.second = a;
            best_w = w;
        }
    }
    
    return R;
}


pair<int,double> Polygon::sideOfPosition(double t) const
{
    double accum=0.0;
    int n = L.size();
    for(int i=0; i<n; i++)
    {
        int j=(i+1)%n;
        double  d = (L[j]-L[i]).mag(),
                w = accum+d;
        if(w > t)
            return pair<int,double>(i, (t-accum)/d);
        accum = w;
    }
    
    return pair<int,double>(0,fmod(t,accum));
}


void Polygon::continuousOverlapInfo( vector<double>& L, double perimeter, OverlapInfo* cInfo ) const
{
    pair<double,double> R = findShortestEncapsulatingInterval( L, perimeter );
    double a = R.first, b = R.second;
    pair<int,double> p;
    
    p = sideOfPosition(a);
    int i = p.first;
    double ti = p.second;
    
    p = sideOfPosition(b);
    int j = p.first;
    double tj = p.second;
    
    continuousOverlapInfo(i,ti,j,tj,cInfo);
}



/*continuousCollisionInfo takes in endpoints of an interval on a polygon as a, b  which are the numbers of the
segments of the polygon that the end points are on, and also the t values of the position 
of the points on the segments, returns*/

void Polygon::continuousOverlapInfo( int a, double ta, int b, double tb, OverlapInfo* cInfo ) const
{
    int size = L.size();
    
    Vec2 A((1-ta)*L[a] + ta*L[(a+1)%size]);
    Vec2 B((1-tb)*L[b] + tb*L[(b+1)%size]);
    Vec2 dA = L[(a+1)%size], dB = L[b];
    double Blength((B-dB).mag()), Alength((dA-A).mag());
    
    // If a==b, we handle separately.  Then the length of the domain of integration is
    // just the distance between A and B
    
    if( a==b )
    {
        cInfo->normal = normalize(right(dA-dB));
        cInfo->contactPoint = 0.5*(A+B);
        return;
    }
    
    // If a!=b, we go through the segments integrating as we go.
    
    Vec2 N, P;
    double length;
    
    // first the segments on the ends of the domain:
    
    length = Alength + Blength;
    P += ((dA+A)*Alength + (dB+B)*Blength);
    
    // then loop through the segments on the interior adding using the whole length of each segment
    for( int i=(a+1)%size; i!=b; i++, i%=size )
    {
        Vec2 C(L[i]), dC(L[(i+1)%size]);
        Vec2 Ccont(dC-C);
        double Clength = Ccont.mag();
        length += Clength;
        P += (dC+C)*Clength;
    }
    
    cInfo->normal = normalize(right(B-A));
    cInfo->contactPoint = 0.5*(P / length);
}


vector<Polygon> Polygon::triangulate() const
{
    /*This algorithm is coded inefficiently
      It should not be called frequently.*/
         
    vector<Polygon> R;
    vector<Vec2> temp_L = L;
    
    int i,j;
    
    while( temp_L.size() > 3 )
    {
        int size = temp_L.size();
        for( i=0; i<size; i++ )
        {
            Vec2 P1 = temp_L[i];
            Vec2 P2 = temp_L[(i+1)%size];
            Vec2 P3 = temp_L[(i+size-1)%size];
            
            Polygon T;
            T.add(P1).add(P2).add(P3);
            
            bool bad_flag = false;
            
            if( Vec3(P2-P1, 0.0).cross(Vec3(P3-P2, 0.0)).z < 0.0 )
            {
                bad_flag = true;
            }
            else
            {
                for( j=0; j<size; j++ )
                {
                    if( j!=i &&
                        (i+1)%size != (j+1)%size &&
                        (j+size-1)%size != (i+size-1)%size )
                    {
                        if( T.vectorInside(temp_L[j]) )
                        {
                            bad_flag = true;
                            break;
                        }
                    }
                }
            }
            
            if(!bad_flag || i==size-1) //just so it doesn't crash
            {
                R.push_back(T);
                vector<Vec2> newL;
                for( int j=0; j<size; j++ )
                {
                    if( j!=i )
                        newL.push_back(temp_L[j]);
                }
                temp_L=newL;
                break;
            }
        }
    }
    
    Polygon T;
    for( unsigned int i=0; i<temp_L.size(); i++ ) T.add(temp_L[i]);
    R.push_back(T);
    
    return R;
}


bool Polygon::simple() const
{
    int size = L.size();
    for(int i=0; i<size; i++)
    {
        int j=(i+1)%size;
        Line2d A(L[i], L[j]);
        
        for(int k=0; k<i; k++)
        {
            int l=(k+1)%size;
            
            if( j!=k && l!=i )
            {
                Line2d B(L[k], L[l]);
                
                vector<double> D(A.intersectLine(B));
                
                if( D[0] >= 0.0 && D[0] <= 1.0 && D[1] >= 0.0 && D[1] <= 1.0 )
                    return false;
            }
        }
    }
    
    return true;
}


int Polygon::windingNumber() const
{
    int size = L.size();
    double theta = 0.0;
    for(int i=0; i<size; i++)
    {
        int j=(i+1)%size, k=(i+2)%size;
        theta += angleTo(L[j]-L[i], L[k]-L[j]);
    }
    
    double t = 0.5*theta*ONE_OVER_PI;
    if( t>0 ) t+=0.5; else t-=0.5;
    return (int)t;
}



void Circle::set( const Vec2& inc, double inr ) {
    tag = kCircle;
    r = inr; C = inc;
}

void Circle::set( double inx, double iny, double inr ) {
    tag = kCircle;
    r = inr; C = Vec2(inx,iny);
}

bool Circle::vectorInside( const Vec2 V ) const {
    return ( (V-C).magSquared()<r*r );
}


class Circle& Circle::operator -= (const Vec2& V) {
    C-=V;
    return *this;
}

class Circle& Circle::operator += (const Vec2& V) {
    C+=V;
    return *this;
}

class Circle Circle::operator* (double k) const {
	Circle C(*this);
	C.C*=k;
	C.r*=k;
	return C;
}

class Circle Circle::operator/ (double k) const {
	Circle C(*this);
	C.C*=k;
	C.r*=k;
	return C;
}

void Circle::print() const
{
    printf("C = <%f, %f>, r = %f", C.x, C.y, r);
}

void Circle::display() const
{
    print();
    printf("\n");
}

void Circle::draw() const
{
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

Vec2 Circle::pointIn() const
{
    return C;
}

void Circle::rotate(double theta)
{
    lib2d::rotate(C, theta);
}

bool Circle::overlaps( const class Circle& inC, OverlapInfo* oInfo ) const
{
    //bounding box check
    Vec2 U = inC.C - C;
    double r_sum = inC.r + r;
    if (( U.x > r_sum ) ||
        ( U.y > r_sum ) ||
        ( U.x < -r_sum ) ||
        ( U.y < -r_sum )) return false;
    //end bounding box check
    
    double d = (r+inC.r);
    bool retval = ( d*d - (inC.C-C).magSquared() > 0.0 );
    if( oInfo && retval )
        continuousOverlapInfo(inC, oInfo);
    
    return retval;
}

bool Circle::overlaps( const class Polygon& P, OverlapInfo* oInfo ) const
{
    bool retval = P.overlaps( *this, oInfo );

    if( oInfo )
        oInfo->normal *=-1;

    return retval;
}

void Circle::continuousOverlapInfo( const class Circle& inC, OverlapInfo* oInfo ) const
{
    oInfo->normal = normalize(inC.C-C);
    oInfo->contactPoint = (inC.r*C + r*inC.C)/(inC.r + r);
}

const vector<double> Circle::intersectLine( const Vec2& P, const Vec2& Q ) const
{
    double a,b,c;

    a = (P.x-Q.x)*(P.x-Q.x) + (P.y-Q.y)*(P.y-Q.y);
    b = 2.0*((P.x-Q.x)*(C.x-P.x) + (P.y-Q.y)*(C.y-P.y));
    c = (P.x-C.x)*(P.x-C.x) + (P.y-C.y)*(P.y-C.y) - r*r;

    vector<double> R(2);

    double q = -0.5*( b + sgn(b)*sqrt(b*b - 4.0*a*c) );
    R[0] = q/a;
    R[1] = c/q;

    return R;
}

shape_type shape_then(const class Shape* A, const class Shape* B) {
    return (shape_type)(((int)(A->tag))*((int)kBase)+((int)(B->tag)));
}

shape_type shape_then(const class Shape& A, const class Shape& B) {
    return shape_then(&A,&B);
}

} //end namespace


