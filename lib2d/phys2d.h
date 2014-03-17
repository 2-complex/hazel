#ifndef _PHYS2D_
#define _PHYS2D_

#ifndef GL_ON
#define GL_ON 1
#endif

#ifndef _MATH2D_
#include "math2d.h"
#endif

#if GL_ON
#include <glut/glut.h>
#endif

#include <pthread.h>

#include "pooledlist.h"

#define kImmutableMass 6.0e250
#define kImmutableRotationalInertia 6.0e250
//(i.e. really big)

namespace lib2d
{

class Delta
{
public:
    Delta()
    {
        t = 0;
        delta_w = 0;
        delta_v.set(0,0);
    }
    
    Delta( double inT, const Vec2& in_delta_v, double in_delta_w )
    {
        t = inT;
        delta_v = in_delta_v;
        delta_w = in_delta_w;
    }
    
    double t;
    Vec2 delta_v;
    double delta_w;
    
    bool operator < (const class Delta& I) const {
        return t < I.t  || (t==I.t && this < &I);
    }
    
    bool operator == (const class Delta& I) const {
        return this == &I;
    }
    
    void print() const {
        printf( "%f", t ); delta_v.print(); printf( "%f", delta_w );
    }
    
    void display() const {
        print(); printf( "\n" );
    };
};



class Collision
{
public:
    Collision(){}
    
    Vec2 pointOfImpact;
    Vec2 normalOfImpact;
    double timeOfImpact;
        
    class Body* P;
    class Body* Q;
    
    bool operator < (const Collision& C) const;
    bool operator == (const Collision& C) const;
    void print() const;
    
    bool penetrates() const;
    void reverse();
    void backUp();
};


typedef enum piece_type {
    kCirclePiece = 0x01,
    kLinePiece = 0x02
} piece_type;



class Piece {
public:
    piece_type tag;
//    bool isCircle;
    Vec2  P,  Q;
    double r;
    class Body* B;
    mutable class Piece* mother;
    double arcA, arcB;
    double perimeterAccum;
    
    inline Piece() {}
    
    inline Piece(const class Piece& p)
    {
        tag = p.tag;
        P.x = p.P.x;
        P.y = p.P.y;
        Q.x = p.Q.x;
        Q.y = p.Q.y;
        r = p.r;
        B = p.B;
        mother = p.mother;
        arcA = p.arcA;
        arcB = p.arcB;
        perimeterAccum = p.perimeterAccum;
    }
    
    inline class Piece& operator =(const class Piece& p)
    {
        tag = p.tag;
        P.x = p.P.x;
        P.y = p.P.y;
        Q.x = p.Q.x;
        Q.y = p.Q.y;
        r = p.r;
        B = p.B;
        mother = p.mother;
        arcA = p.arcA;
        arcB = p.arcB;
        perimeterAccum = p.perimeterAccum;
        return *this;
    }
    
    
    Piece(Vec2 inP, Vec2 inQ, Body* inB, double inArcA, double inArcB, double inPerimeterAccum)
    {
        tag = kLinePiece;
        P = inP;
        Q = inQ;
        B = inB;
        arcA = inArcA;
        arcB = inArcB;
        perimeterAccum = inPerimeterAccum;
    }
    
    Piece(Vec2 inP, double inr, Body* inB)
    {
        tag = kCirclePiece;
        P = inP;
        r = inr;
        B = inB;
        arcA = 0;
        arcB = 2.0*PI*r;
        perimeterAccum = 0.0;
    }
    
    typedef PooledList<Piece> PooledPieceList;
    
    bool overlaps( const Piece* piece, vector<double>& where ) const;
    void onLeftOnRight( const Vec2& K, const Vec2& D, bool* on_left, bool* on_right ) const;
    Vec2 midpoint() const;
    void cutAndDivvy( Vec2 K1, Vec2 D, PooledPieceList& leftPieces, PooledPieceList& rightPieces ) const;
    
    void print() const
    {
        printf( "%p ", B );
        switch(tag)
        {
            case kCirclePiece:
                printf( "Circle: " );
                P.print();
                printf( ", %f", r );
                printf( "  from %f to %f", arcA, arcB );
            break;
            
            case kLinePiece:
                printf( "Line: " );
                P.print();
                Q.print();
                printf( "  from %f to %f", arcA, arcB );
            break;
            
            default:
                assert(false);
            break;
        }
    }
    
    void display() const
    {
        print();
        printf( "\n" );
    }
    
    void draw() const;
};

typedef Piece::PooledPieceList PooledPieceList;


bool bodiesCollide( const class Body* P, const class Body* Q );
bool bodiesOverlap( const class Body* P, const class Body* Q, OverlapInfo* oInfo );
bool bodiesOverlap( const class Body* P, const class Body* Q );
bool shapesOverlap( const Shape* P, const Shape* Q, OverlapInfo* oInfo = NULL );
Vec2 shapePointIn(const Shape* s);



class Body {

public:
    Vec2 position;
    Vec2 velocity;
    double theta; //rotation
    double omega; //rotational velocity
    double restitution;
    double dynamicFriction;
    double staticFriction;
    double mass;
    double rotationalInertia;
    Vec2 center;
    
    double rememberedMass;
    double rememberedRotationalInertia;
    
private:
    Vec2 old_position;
    double old_theta;
    
public:
    
    bool piecesValid;
    int numberOfPieces;
    Piece* pieces;
    
    void updatePieces();
    void printPieces();
    
    char* name;
    
    bool immutable;
    bool nailed;
    bool erase_me;
    
public:
    
    Body(const class Circle& inc) {
        basic_shape = new Circle(inc);
        s = new Circle();
        numberOfPieces = 1;
        pieces = new Piece[1];
        
        name = NULL;
        tare();
    }
    
    Body(const class Polygon& inp) {
        basic_shape = new Polygon(inp);
        s = new Polygon();
        numberOfPieces = inp.points().size();
        pieces = new Piece[inp.points().size()];
        
        name = NULL;
        tare();
    }
    
    Body()
    {
        s = basic_shape = NULL;
        pieces = NULL;
        name = NULL;
        tare();
    }
    
    virtual ~Body()
    {
        if( basic_shape ){ delete basic_shape; basic_shape = NULL; }
        if( s ) { delete s; s = NULL; }
        if( pieces ) { delete[] pieces; pieces = NULL; }
    }
    
    void makeImmutable()
    {
        if( !immutable )
        {
            if( !nailed )
                rememberedMass = mass;
            
            rememberedRotationalInertia = rotationalInertia;
            
            mass = kImmutableMass;
            rotationalInertia = kImmutableRotationalInertia;
            velocity.set(0,0);
            omega = 0.0;
        }
        
        immutable = true;
    }
    
    void makeMutable()
    {
        if( immutable )
        {
            rotationalInertia = rememberedRotationalInertia;
            if( !nailed )
                mass = rememberedMass;
        }
        
        immutable = false;
    }
    
    void nail()
    {
        if( !nailed && !immutable )
        {
            rememberedMass = mass;
            mass = kImmutableMass;
            velocity.set(0,0);
        }
        
        nailed = true;
    }
    
    void unnail()
    {
        if( nailed )
        {
            if( !immutable )
                mass = rememberedMass;
        }
        
        nailed = false;
    }
    
    void markForRemoval()
    {
        erase_me = true;
    }
    
    virtual const char* getName() const {return name;}
    
    void applyDelta(const Delta& D);
    void move(double u);
    void moveBack();
    void tare();
            
    void set(const class Circle& C);
    void set(const class Polygon& P);
    
    inline class Body& operator += (const class Vec2& V) {
        position += V;
        return *this;
    }
    
    inline class Body& operator -= (const class Vec2& V) {
        position -= V;
        return *this;
    }
    
    inline class Body& rotate( double delta ) {
        theta += delta;
		return *this;
    }

    virtual void draw() {
#ifdef GL_ON
        adjustShape();
	
        switch(s->tag)
        {
            case kCircle: ((Circle*)(s))->draw(); break;
            case kPolygon: ((Polygon*)(s))->draw(); break;
            default: assert(false); break;
        }
#endif
    }

    void display() const;
    void print() const;
    
    inline void adjustShape() const
    {
        switch(s->tag)
        {
            case kCircle:
                (*((Circle*)(s))) = (*((Circle*)(basic_shape)));
                (*((Circle*)(s)))-=center;
                ((Circle*)(s))->rotate(theta);
                (*((Circle*)(s)))+=position;
            break;
            case kPolygon:
                (*((Polygon*)(s))) = (*((Polygon*)(basic_shape)));
                (*((Polygon*)(s)))-=center;
                ((Polygon*)(s))->rotate(theta);
                (*((Polygon*)(s)))+=position;
            break;
            
            default:
            break;
        }
    }

    bool overlaps( const Body& B, OverlapInfo* oInfo = NULL ) const;
    
    mutable class Shape* s;
    class Shape* basic_shape;
};



Mat3 KMatrix(double m, double I, Vec3 r);

Vec3 compute_impulse_from_Vr(
    Vec3 Vr,
    double m1, double I1, Vec3 r1,
    double m2, double I2, Vec3 r2,
    Vec3 N, double e );

Vec3 compute_impulse(
    Vec3 v1, Vec3 w1, double m1, double I1, Vec3 r1,
    Vec3 v2, Vec3 w2, double m2, double I2, Vec3 r2,
    Vec3 N, double e );
                            
Vec3 compute_impulse_with_friction(
    Vec3 v1, Vec3 w1, double m1, double I1, Vec3 r1,
    Vec3 v2, Vec3 w2, double m2, double I2, Vec3 r2,
    Vec3 N, double e, double u_s, double u_d );



typedef enum {
    kMaximum=0,
    kMinimum=1,
    kAverage=2
} ComputeType;


class Universe {
public:
    list<Body*> L;
    Vec2 gravity;
    double surfaceFriction;
    
private:
    bool bail;
    
    typedef bool (*overlap_function) (const class Body*, const class Body*);
    typedef pair<Delta, Delta> (*collision_function) (const Collision&);
    
    overlap_function overlap;
    
public:
    
    ComputeType frictionComputeType;
    ComputeType restitutionComputeType;
    
    double impulseJitter;
    double positionJitter;
    double velocityJitter;
    bool adaptiveTimeStepOn;
    double timeStepFactor;
    double minimumTimeStep;
    bool nudgeForceOn;
    double nudgeForceAmmount;
    int iterationsBeforeForcedRestitution;
    
    double pushOutNormalJitter;
    bool forceRestitution;
    double forcedRestitution;

    
    Universe() {
        bail = false;
        threading = false;
        overlap = bodiesOverlap;
        surfaceFriction = 0.0;
        timeStep = 1.0;
        
        pushOutNormalJitter = 0.01;
        forcedRestitution = 1.1;
        forceRestitution = false;
        
        impulseJitter = 0.0001;
        positionJitter = 0.0001;
        velocityJitter = 0.0001;
        adaptiveTimeStepOn = false;
        timeStepFactor = 1.0;
        minimumTimeStep = 0.1;
        nudgeForceOn = false;
        nudgeForceAmmount = 0.001;
        iterationsBeforeForcedRestitution = 5;
    }
    
    ~Universe() {
    }
    
    void clear();

public:
    void draw() const
    {
#if GL_ON
        list<Body*>::const_iterator list_itr;
        for( list_itr = L.begin(); list_itr != L.end(); list_itr++ )
        {
            (*list_itr)->draw();
        }
#endif
    }
    
    class Universe& add(Body& B) {return add(&B);};
    class Universe& add(Body* B);
    
    void removeAllMarkedBodies();
    
private:
    mutable PooledPieceList::pooltype pool;
    void adjustAllShapes();
    
public:
    void display() const {print(); printf("\n");}
    void print() const;
    
    void printBodyPieces() const;
    
    double timeStep;
    
    void move();
    void move(double t);
    void moveBack();
    void applyGravity();
    void applySurfaceFriction();
    
    void binarySpaceSearch( PooledPieceList& pieceList, PooledPieceList::pooltype& pool, int draw_step = 0 );
    void binarySpaceSearch_helper( PooledPieceList& inL, int depth, PooledPieceList::pooltype& pool, int draw_step = 0 );
    void binarySpaceSearchDiscrete_helper(PooledPieceList& inL, int depth, PooledPieceList::pooltype& pool, int draw_step = 0);
    
    map< pair<Body*, Body*> ,vector<double> > bodyMap;
    set<pair<Piece*, Piece*> > piecePairs;
    
    void bailOut();
    void bailIn();
    
    pair<Delta,Delta> bounce(const Collision& C);
    void handleCollisions();
    bool crudeCollisionDetection( Body* A, Body* B, Collision* C );
    void rebuildPiecePairs(int draw_step = 0);
    set<Collision> findCollisionsInPiecePairs();
    set<Collision> findAllCollisionsBinary();
    set<Collision> findAllCollisions();
    bool findMoreCollisionsInPiecePairs(set<Collision>& c_set);
    bool findMoreCollisionsBinary(set<Collision>& c_set);
    PooledPieceList makePieceList() const;
    
    void drawAllCollisions();
    void drawAllCollisionsBinary();
    
    void drawCollisionSet(const set<Collision> &S, double arrowLength);
    
    void drawHalfs();
    void drawLeaves();
    
    void pushAllOut();
    void pushPiecePairsOut();
    void pushFromBodyMap();
    
    void naturalPushOut( Body& P, Body& Q );
    
    void activateThreading();
    
    bool threading;
    pthread_mutex_t bodyMutex;
    pthread_mutex_t bailMutex;
};


}  //end namespace

#endif




