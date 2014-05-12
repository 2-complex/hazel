#include <algorithm>

#include "math2d.h"
#include "phys2d.h"
#include "pooledlist.h"

#include <stack>

#include <pthread.h>

#if GL_ON
#include "opengl.h"
#endif


namespace lib2d
{

void Body::tare()
{
    position.x = position.y = velocity.x = velocity.y = theta = omega = 0.0;
    staticFriction = 0.8;
    dynamicFriction = 0.6;
    mass = 10.0;
    rotationalInertia = 10000.0;
    restitution = 0.5;
    stiff = false;
    nailed = false;
    erase_me = false;
    
    piecesValid = false;
}


void Body::set(const Circle& C)
{
    if( basic_shape ) delete basic_shape;
    if( s ) delete s;
    if( pieces ) delete[] pieces;
    
    basic_shape = new Circle(C);
    s = new Circle();
    numberOfPieces = 1;
    pieces = new Piece[1];
    
    tare();
}

void Body::set(const Polygon& P)
{
    if( basic_shape ) delete basic_shape;
    if( s ) delete s;
    if( pieces ) delete[] pieces;
    
    basic_shape = new Polygon(P);
    s = new Polygon();
    numberOfPieces = P.points().size();
    pieces = new Piece[P.points().size()];
    
    tare();
}

void Body::draw() const
{
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


void Body::display() const
{
    print();
    printf("\n");
}

void Body::print() const
{
    printf( "shape: " );
    adjustShape();
    switch(s->tag)
    {
        case kCircle: ((Circle*)(s))->print(); break;
        case kPolygon: ((Polygon*)(s))->print(); break;
        default: assert(false); break;
    }
    printf( "\n" );
    
    printf( "position: " );
    position.print();
    printf( "\n" );
    
    printf( "velocity: " );
    velocity.print();
    printf( "\n" );
    
    printf( "center:" );
    center.print();
    printf( "\n" );
    
    printf( "theta: %f\nomega: %f\n", theta, omega );
    printf( "mass: %f\nrotationalInertia: %f\n", mass, rotationalInertia );
}


void Body::applyDelta(const Delta& D)
{
    omega += D.delta_w;
    velocity += D.delta_v;
}

void Body::move(double u)
{
    old_position = position;
    old_theta = theta;
    
    position += velocity*u;
    theta += omega*u;
}

void Body::moveBack()
{
    position = old_position;
    theta = old_theta;
}


bool Body::overlaps( const Body& B, OverlapInfo* oInfo ) const {
    bool retval = false;
    
    if(&B == this)
        return true;
    else
    {
        adjustShape();
        B.adjustShape();
        retval = shapesOverlap( s, B.s, oInfo );
    }
    
    return retval;
}


bool Collision::operator < (const Collision& C) const
{
    return (timeOfImpact < C.timeOfImpact) || (timeOfImpact == C.timeOfImpact && this < &C);
}

bool Collision::operator == (const Collision& C) const
{
    return  P==C.P && Q==C.Q &&
            pointOfImpact == C.pointOfImpact &&
            normalOfImpact == C.normalOfImpact &&
            timeOfImpact == C.timeOfImpact;
}

void Collision::print() const
{
    //printf( "Collision: bodies: \n" );
    //P->print();
    //Q->print();
    printf( "when: %f where: ", timeOfImpact );
    pointOfImpact.print();
    printf( " how: " );
    normalOfImpact.display();
}

void Collision::reverse()
{
    normalOfImpact = -normalOfImpact;
}


void Universe::clear()
{
    L.clear();
}

void Universe::draw() const
{
#if GL_ON
    list<Body*>::const_iterator list_itr;
    for( list_itr = L.begin(); list_itr != L.end(); list_itr++ )
        (*list_itr)->draw();
#endif
}

Vec2 randVec2( double amount )
{
    double t = (double)(random());
    double u = (double)(random()%1000001)/1000000.0;
    return amount * u * Vec2( cos(t), sin(t) );
}

Vec3 randVec3( double amount )
{
    double t = (double)(random());
    double u = (double)(random());
    double v = (double)(random()%1000001)/1000000.0;
    return Vec3( sin(u)*cos(t), sin(u)*sin(t), cos(u) ) * amount * v;
}

void Universe::naturalPushOut( Body& P, Body& Q )
{
    Vec2 V, P_v, Q_v;
    double fact=0.1;
    int counter = 0;
    OverlapInfo oInfo;
    
    while( P.overlaps(Q) )
    {
        Vec2 N;
        
        N.set(1,0);
        if( P.overlaps(Q, &oInfo) ) N = (oInfo.normal);
        
        assert(N!=Vec2(0.0,0.0));
        
        //a workaround for a bug in overlaps that makes overlaps return (nan,nan)
        if( !(N.magSquared() > 0.5 && N.magSquared() < 2.0) ) N.set(0,0);
        
        V = fact*(N + randVec2(pushOutNormalJitter));
        
        Vec2 A,B;
        
        //with the big masses, this became necessary:
        if( P.stiff )
        {
            A = -0.001*V; //not exactly 1 and 0, lest pushOutAll run for ever
            B =  0.999*V;
        }
        else if( Q.stiff )
        {
            A = -0.999*V;
            A =  0.001*V;
        }
        else
        {
            A = -V*(Q.mass/(P.mass+Q.mass));
            B =  V*(P.mass/(P.mass+Q.mass));
        }
        
        P += A; P_v += A;
        Q += B; Q_v += B;
        
        fact*=2;
        
        counter++;
    }
    
    P_v*=0.5; Q_v*=0.5;
    while( counter-- )
    {
        Q-=Q_v; P-=P_v;
        if( P.overlaps(Q) ){ Q+=Q_v; P+=P_v; }
        P_v*=0.5; Q_v*=0.5;
    }
}


void Universe::move()
{
    move(timeStep);
}

void Universe::move(double t)
{
    if(threading)
        pthread_mutex_lock(&bodyMutex);
    
    for( list<Body*>::iterator itr = L.begin(); itr!=L.end(); itr++)
        (*itr)->move(t);
    
    if(threading)
        pthread_mutex_unlock(&bodyMutex);
}

void Universe::moveBack()
{
    if(threading)
        pthread_mutex_lock(&bodyMutex);
    
    for( list<Body*>::iterator itr = L.begin(); itr!=L.end(); itr++)
        (*itr)->moveBack();
    
    if(threading)
        pthread_mutex_unlock(&bodyMutex);
}


void Universe::applyGravity()
{
    for( list<Body*>::iterator itr = L.begin(); itr!=L.end(); itr++)
    {
        if( !(*itr)->nailed )
        {
            (*itr)->velocity += gravity*timeStep;
        }
    }
}

void applySurfaceFriction()
{
    /*implement this later*/
}


Vec2 Piece::midpoint() const
{
    double theta;
    double a, b;
    Vec2 V;
    
    switch(tag)
    {
        case kCirclePiece:
            a=arcA;
            b=arcB;
            if(b>a) a+=2.0*PI;
            theta = 0.5*(a+b)/r;
            V.set(r,0);
            lib2d::rotate(V, theta);
            V+=P;
            return V;
        break;
        
        case kLinePiece:
            return 0.5*((P*(1.0-arcA)+Q*arcA) + (P*(1.0-arcB)+Q*arcB));
        break;
        
        default:
            assert(false);
        break;
    }
    
    assert(false);
    return Vec2(0,0);
}



bool Piece::overlaps( const Piece* piece, vector<double>& where ) const
{
    bool retval = false;
    
    Vec2 Pt(P), Qt(Q), pPt(piece->P), pQt(piece->Q);
    double pr = piece->r;
    
    if( tag == kCirclePiece && piece->tag == kCirclePiece )
    {
        //bounding box check
        if( (Pt.x+r < pPt.x-pr) || (Pt.y+r < pPt.y-pr) || (pPt.x+pr < Pt.x-r) || (pPt.y+pr < Pt.y-r) )
            return false;
        
        if( Circle(Pt,r).overlaps(Circle(pPt,pr)) )
        {
            where.push_back( 0.0 );
            return true;
        }
        return false;
    }
    
    if( tag == kCirclePiece && piece->tag == kLinePiece )
        return piece->overlaps(this, where);
    
    
    //information for bounding box check:
    double max_x, max_y, min_x, min_y, pmax_x, pmax_y, pmin_x, pmin_y;
    if( Pt.x < Qt.x ) {max_x = Qt.x; min_x = Pt.x;} else {max_x = Pt.x; min_x = Qt.x;}
    if( Pt.y < Qt.y ) {max_y = Qt.y; min_y = Pt.y;} else {max_y = Pt.y; min_y = Qt.y;}
        
    
    if( tag == kLinePiece && piece->tag == kCirclePiece )
    {
        //bounding box check:
        if( max_x < pPt.x-pr || max_y < pPt.y-pr || pPt.x+pr < min_x || pPt.y+pr < min_y )
            return false;
        
        double D;
        vector<double> R = Circle(pPt,pr).intersectLine(Pt,Qt);
        
        for( int i=0; i<2; i++ )
        {
            if( R[i]*(R[i]-1.0)<=0.0 )
            {	
                D = (Pt-Qt).mag();
                
                where.push_back( perimeterAccum + R[i]*D );
                retval = true;
            }
        }
        return retval;
    }
    
    if( tag == kLinePiece && piece->tag == kLinePiece )
    {
        //for bounding box
        if( pPt.x < pQt.x ) {pmax_x = pQt.x; pmin_x = pPt.x;} else {pmax_x = pPt.x; pmin_x = pQt.x;}
        if( pPt.y < pQt.y ) {pmax_y = pQt.y; pmin_y = pPt.y;} else {pmax_y = pPt.y; pmin_y = pQt.y;}
        
        if( max_x < pmin_x || max_y < pmin_y || pmax_x < min_x || pmax_y < min_y )
            return false;
        
        
        vector<double> R = Line2d(Pt,Qt).intersectLine(Line2d(pPt,pQt));
        if( R[0]*(R[0]-1.0)<=0.0 && R[1]*(R[1]-1.0)<=0.0 )
        {
            double D = (Pt-Qt).mag();
            where.push_back( perimeterAccum + R[0]*D );
            retval = true;
        }
        return retval;
    }
    
    //shouldn't get here
    return false;
}



bool Collision::penetrates() const
{
    double t = timeOfImpact;
    
    Vec2 R1_2d( pointOfImpact - (P->position + t*P->velocity) ),
             R2_2d( pointOfImpact - (Q->position + t*Q->velocity) );
    
    Vec3 r1( R1_2d, 0.0 ), r2( R2_2d, 0.0 );
    
    Vec3 v1(P->velocity, 0.0);
    Vec3 v2(Q->velocity, 0.0);
    Vec3 w1(0.0, 0.0, P->omega);
    Vec3 w2(0.0, 0.0, Q->omega);
    
    Vec3 Vr = (w2.cross(r2) + v2) - (w1.cross(r1) + v1);
    Vec3 N(normalOfImpact, 0.0);
    
    return( Vr.dot(N) <= 0.0 );
}


PooledPieceList Universe::makePieceList() const
{
    PooledPieceList pieceList(pool);
    
    
    for( list<Body*>::const_iterator itr = L.begin(); itr!=L.end(); itr++)
    {
        (*itr)->updatePieces();
        
        int num = (*itr)->numberOfPieces;
        Piece* pieces = (*itr)->pieces;
        for( int i=0; i<num; i++ )
            pieceList.add(pieces[i]);
    }
    
    return pieceList;
}



void Universe::bailOut()
{
    if(threading)
        pthread_mutex_lock(&bailMutex);
    
    bail = true;
    
    if(threading)
        pthread_mutex_unlock(&bailMutex);
}

void Universe::bailIn()
{
    if(threading)
        pthread_mutex_lock(&bailMutex);
    
    bail = false;
    
    if(threading)
        pthread_mutex_unlock(&bailMutex);
}



void Universe::handleCollisions()
{
    srandom(0xc0ffee);
    
    int counter = 0;
    
    forceRestitution = false;
    
    map<Body*, vector<Delta> > deltaMap;
    
    
    set<Collision> S;
    
    while( findMoreCollisionsBinary(S) )
    {
        if(bail) return;
        
        deltaMap.clear();
        for( list<Body*>::iterator itr = L.begin(); itr!=L.end(); itr++ )
            deltaMap[*itr] = vector<Delta>();
        
        if( counter > iterationsBeforeForcedRestitution )
        {
            forceRestitution = true;
            forcedRestitution = sin((PI/6.0)*(counter-iterationsBeforeForcedRestitution));
        }
        
        
        bool okay = false;
        
        while( !okay )
        {
            okay = true;
            for( set<Collision>::iterator itr = S.begin(); itr!=S.end(); itr++ )
            {      
                if(bail) return;
                
                const Collision& C(*itr);
                
                if(C.penetrates())
                {
                    if( !(C.P->immutable() && C.Q->immutable()) )
                    {
                        pair<Delta,Delta> deltaPQ( bounce(C) ); //get the deltas for a bounce
                        
                        if( !(C.P->immutable()) )
                            C.P->applyDelta(deltaPQ.first); //then apply the deltas
                        
                        if( !(C.Q->immutable()) )
                            C.Q->applyDelta(deltaPQ.second);
                    }
                }
            }
            
            for( set<Collision>::iterator itr = S.begin(); itr!=S.end(); itr++ )
            {
                Collision C(*itr);
                if( C.penetrates() ) // if the collision is still penetrating, we plan on doing this again
                {
                    okay = false;
                    break;
                }
            }
        }
        
        counter++;
    }
    
    
    
}


bool Universe::crudeCollisionDetection( Body* A, Body* B, Collision* C )
{
    //detect the collision the crude, old fashioned way.
    //just move it to where it would be, detect overlap, move back.
    A->move(timeStep);
    B->move(timeStep);
    
    OverlapInfo oInfo;
    bool found_one = A->overlaps(*B, &oInfo);
    if( found_one )
    {
        C->P = A;
        C->Q = B;
        C->normalOfImpact = oInfo.normal;
        C->pointOfImpact = oInfo.contactPoint;
        C->timeOfImpact = timeStep;
    }
    
    A->moveBack();
    B->moveBack();
    
    return found_one;
}


void Universe::rebuildPiecePairs(int draw_step)
{
    pool.clear();
    
    move();
    PooledPieceList pieceList( makePieceList() );
    moveBack();
        
    binarySpaceSearch(pieceList, pool, draw_step);
}


set<Collision> Universe::findCollisionsInPiecePairs()
{
    bodyMap.clear();
    
    for( set<pair<Piece*, Piece*> >::iterator pair_itr = piecePairs.begin(); pair_itr != piecePairs.end(); pair_itr++ )
    {
        Piece* A = pair_itr->first;
        Piece* B = pair_itr->second;
        
        if( A->B == B->B ) continue;
        if( A->B > B->B ) { Piece* buff = A; A = B; B = buff; }
        
        vector<double> where;
        
        if( A->overlaps(B, where) )
        {
            for( vector<double>::iterator itr = where.begin(); itr!=where.end(); itr++ )
                bodyMap[pair<Body*,Body*>(A->B, B->B)].push_back(*itr);
        }
    }
    
    
    set<Collision> c_set;
    
    for( map< pair<Body*, Body*>, vector<double> >::iterator itr = bodyMap.begin(); itr!=bodyMap.end(); itr++ )
    {
        Collision C;
        
        Body* A=(itr->first.first);
        Body* B=(itr->first.second);
        vector<double> where = itr->second;
        OverlapInfo oInfo;
        
        Circle* circleA;
        Circle* circleB;
        Polygon* polygonA;
        Polygon* polygonB;
        circleA = (Circle*)(A->s);
        circleB = (Circle*)(B->s);
        polygonA = (Polygon*)(A->s);
        polygonB = (Polygon*)(B->s);
        
        switch( shape_then(A->s, B->s) )
        {            
            case kCircleThenCircle:
                circleA->continuousOverlapInfo( *circleB, &oInfo );
            break;
            
            case kPolygonThenCircle:
                polygonA->continuousOverlapInfo( where, polygonA->perimeter(), &oInfo );
            break;
            
            case kCircleThenPolygon:
                polygonB->continuousOverlapInfo( where, polygonB->perimeter(), &oInfo );
                oInfo.normal = -oInfo.normal;
            break;
            
            case kPolygonThenPolygon:
                polygonA->continuousOverlapInfo( where, polygonA->perimeter(), &oInfo );
            break;
            
            default:
                assert(false);
            break;
        }
        
        C.P = A;
        C.Q = B;
        C.normalOfImpact = oInfo.normal;
        C.pointOfImpact = oInfo.contactPoint;
        C.timeOfImpact = timeStep;
        
        if( C.penetrates() )
            c_set.insert(C);
    }
    
    return c_set;
}




bool Universe::findMoreCollisionsInPiecePairs(set<Collision>& c_set)
{
    bool retval = false;
    
    bodyMap.clear();
    
    for( set<pair<Piece*, Piece*> >::iterator pair_itr = piecePairs.begin(); pair_itr != piecePairs.end(); pair_itr++ )
    {
        if(bail) return retval;
        
        Piece* A = pair_itr->first;
        Piece* B = pair_itr->second;
        
        if( A->B == B->B ) continue;
        if( A->B > B->B ) { Piece* buff = A; A = B; B = buff; }
        
        vector<double> where;
        
        if( A->overlaps(B, where) )
        {
            for( vector<double>::iterator itr = where.begin(); itr!=where.end(); itr++ )
                bodyMap[pair<Body*,Body*>(A->B, B->B)].push_back(*itr);
        }
    }
    
    for( map< pair<Body*, Body*>, vector<double> >::iterator itr = bodyMap.begin(); itr!=bodyMap.end(); itr++ )
    {
        if(bail) return retval;
        
        Collision C;
        
        Body* A=(itr->first.first);
        Body* B=(itr->first.second);
        vector<double> where = itr->second;
        OverlapInfo oInfo;
        
        Circle* circleA;
        Circle* circleB;
        Polygon* polygonA;
        Polygon* polygonB;
        circleA = (Circle*)(A->s);
        circleB = (Circle*)(B->s);
        polygonA = (Polygon*)(A->s);
        polygonB = (Polygon*)(B->s);
        
        switch( shape_then(A->s, B->s) )
        {            
            case kCircleThenCircle:
                circleA->continuousOverlapInfo( *circleB, &oInfo );
            break;
            
            case kPolygonThenCircle:
                polygonA->continuousOverlapInfo( where, polygonA->perimeter(), &oInfo );
            break;
            
            case kCircleThenPolygon:
                polygonB->continuousOverlapInfo( where, polygonB->perimeter(), &oInfo );
                oInfo.normal = -oInfo.normal;
            break;
            
            case kPolygonThenPolygon:
                polygonA->continuousOverlapInfo( where, polygonA->perimeter(), &oInfo );
            break;
            
            default:
                assert(false);
            break;
        }
        
        C.P = A;
        C.Q = B;
        C.normalOfImpact = oInfo.normal;
        C.pointOfImpact = oInfo.contactPoint;
        C.timeOfImpact = timeStep;
        
        if( C.penetrates() )
        {
            retval = true;
            c_set.insert(C);
        }
    }
    
    return retval;
}



bool Universe::findMoreCollisionsBinary(set<Collision>& c_set)
{
    rebuildPiecePairs();
    return findMoreCollisionsInPiecePairs(c_set);
}


set<Collision> Universe::findAllCollisionsBinary()
{
    rebuildPiecePairs();
    return findCollisionsInPiecePairs();
}

set<Collision> Universe::findAllCollisions()
{
    set<Collision> c_set;
    for( list<Body*>::iterator itrB = L.begin(); itrB != L.end(); itrB++ )
    for( list<Body*>::iterator itrA = L.begin(); itrA != itrB; itrA++ )
    {
        //it's so crazy it just might work.
        Collision C;
        bool found_one = crudeCollisionDetection( *itrA, *itrB, &C );
        
        if( found_one && C.penetrates() )
        {
            c_set.insert(C);
            continue;
        }
    }
    
    return c_set;
}



void Universe::pushAllOut()
{
    bool clear_flag;
    
    do
    {
        clear_flag = true;
        
        for( list<Body*>::iterator itrB = L.begin(); itrB != L.end(); itrB++ )
        for( list<Body*>::iterator itrA = L.begin(); itrA != itrB; itrA++ )
        {
            if( (*itrA)->overlaps(**itrB) )
            {                
                clear_flag = false;
                naturalPushOut(**itrA,**itrB);
            }
        }
    } while( !clear_flag );
}


void Universe::pushPiecePairsOut()
{
    for( set<pair<Piece*, Piece*> >::iterator pair_itr = piecePairs.begin(); pair_itr != piecePairs.end(); pair_itr++ )
    {
        Piece* A = pair_itr->first;
        Piece* B = pair_itr->second;
        
        if( A->B == B->B ) continue;
        
        naturalPushOut( *(A->B), *(B->B) );
    }
}

void Universe::pushFromBodyMap()
{
    for( map< pair<Body*, Body*> ,vector<double> >::iterator itr = bodyMap.begin(); itr!=bodyMap.end(); itr++ )
        naturalPushOut( *(itr->first.first), *(itr->first.second) );
}




void Universe::print() const
{
    int n=1;
    printf( "******\n" );
    for( list<Body*>::const_iterator itr = L.begin(); itr!=L.end(); itr++)
    {
        printf( "%d:\n", n++ );
        (*itr)->display();
    }
}

void Universe::printBodyPieces() const
{
    int n=1;
    for( list<Body*>::const_iterator itr = L.begin(); itr!=L.end(); itr++)
    {
        printf( "%d:\n", n++ );
        (*itr)->printPieces();
        printf( "\n" );
    }
}


Universe& Universe::add(Body* B)
{   
    L.push_back(B);
    return *this;
}

void Universe::adjustAllShapes()
{
    for( list<Body*>::iterator itr = L.begin(); itr!=L.end(); itr++)
        (*itr)->adjustShape();
}


void Universe::removeAllMarkedBodies()
{
    list<Body*>::iterator next = L.begin();
    for( list<Body*>::iterator itr = next; itr != L.end(); itr = next )
    {
        next++;
        if( (*itr)->erase_me )
        {
            L.erase(itr);
            (*itr)->erase_me = false;
        }
    }
}



Vec2 shapePointIn(const Shape* s)
{
    switch(s->tag)
    {
        case kCircle:   return ((Circle*)s)->pointIn(); break;
        case kPolygon:
            return ((Polygon*)s)->pointIn();
        break;
        
        default:
            assert(false);
        break;
    }
    
    assert(false);
    return Vec2();
}


bool bodiesOverlap( const class Body* P, const class Body* Q )
{
    return bodiesOverlap(P,Q,NULL);
}

bool bodiesOverlap( const Body* P, const Body* Q, OverlapInfo* oInfo )
{    
    P->adjustShape();
    Q->adjustShape();
    return shapesOverlap( P->s, Q->s, oInfo );
}

bool shapesOverlap( const Shape* P, const Shape* Q, OverlapInfo* oInfo )
{
    if( oInfo )
        oInfo->tare();
    
    switch(shape_then(P,Q))
    {
        case kCircleThenCircle:    return ((Circle*)P)->overlaps( *((Circle*)Q), oInfo ); break;
        case kCircleThenPolygon:   return ((Circle*)P)->overlaps( *((Polygon*)Q), oInfo ); break;
        case kPolygonThenCircle:   return ((Polygon*)P)->overlaps( *((Circle*)Q), oInfo ); break;
        case kPolygonThenPolygon:  return ((Polygon*)P)->overlaps( *((Polygon*)Q), oInfo ); break;
        default:
            assert(false);
            return true; // shouldn't make it here.
        break;
    }
    assert(false);
    return true; // shouldn't ever make it here.
}



void Body::updatePieces()
{
    adjustShape();
    Vec2 P;
    
    /*bug workaround: (things passing through eachother):*/
    piecesValid = false;
    
    if( piecesValid )
    {
        switch(s->tag)
        {
            case kCircle:
                pieces[0].P = ((Circle*)(s))->C;
                pieces[0].r = ((Circle*)(s))->r;
            break;
            
            case kPolygon:
            {   
                const vector<Vec2>& PL( ((Polygon*)(s))->points() );
                int size = numberOfPieces;
                
                int i=0;
                for( vector<Vec2>::const_iterator itr = PL.begin(); itr!=PL.end(); itr++, i++ )
                {
                    pieces[(i+size-1)%size].Q = pieces[i].P = *itr;
                }
            }
            break;
            
            default:
            break;
        }
    }
    else
    {
        switch(s->tag)
        {
            case kCircle:
            {
                P = (((Circle*)(s))->C);
                pieces[0] = Piece( P, ((Circle*)(s))->r, this );
                pieces[0].mother = pieces;
            }
            break;
            
            case kPolygon:
            {   
                const vector<Vec2>& PL( ((Polygon*)(s))->points() );
                int size = numberOfPieces;
                
                double accum = 0.0, newaccum;
                for( int i=0; i<size; i++ )
                {
                    newaccum = accum + (PL[i] - PL[(i+1)%size]).mag();
                    pieces[i] = Piece( PL[i], PL[(i+1)%size], this, 0.0, 1.0, accum );
                    pieces[i].mother = pieces+i;
                    
                    accum = newaccum;
                }
            }
            break;
			
            default:
            break;
        }
        
        piecesValid = true;
    }
}

void Body::printPieces()
{
    for( int i=0; i<numberOfPieces; i++ )
    {
        pieces[i].print();
    }
}




struct SearchHelperStackItem {
    PooledPieceList L;
    int depth;
};

void Universe::binarySpaceSearch(PooledPieceList& pieceList, PooledPieceList::pooltype& pool, int draw_step)
{
    binarySpaceSearchDiscrete_helper(pieceList, 16, pool, draw_step);
}

void Universe::binarySpaceSearchDiscrete_helper(PooledPieceList& inL, int depth, PooledPieceList::pooltype& pool, int draw_step)
{
    SearchHelperStackItem* myStack = (SearchHelperStackItem*)malloc( sizeof(SearchHelperStackItem)*3*(depth+5) );
    int myStackPtr = 0;
    myStack[myStackPtr].L = inL;
    myStack[myStackPtr].depth = depth;
    
    Vec2 K1,K2,D,P,Q;
    vector<double> R(2), A(2), B(2);
    Piece p;
    int mydepth;
    
    int step = 0;
    
    piecePairs.clear();
    
    
    for(;myStackPtr+1; myStackPtr--)
    {
        step++;
        
        PooledPieceList& L(myStack[myStackPtr].L);
        mydepth = myStack[myStackPtr].depth;
        
        // If there's only one body represented in the list of pieces,
        // we can move on.  If there are two, we use that information to pick a good K and D
        
        if( L.size() < 2 ) continue; // empty lists and singletons are trash for sure!
        
        //if everybody in town is immutable we can prune
        bool safe_to_prune = true;
        
        double rect_left=1.0e+200, rect_right=-1.0e+200,   rect_top=-1.0e+200, rect_bottom=1.0e+200;
        
        for( PooledPieceList::Node* n = L.begin(); n!=L.end(); n = n->next )
        {
            if( !(n->datum.B->immutable()) )
            {
                safe_to_prune = false;
                break;
            }
        }
        
        if(safe_to_prune) continue;
        
        
        bool two_bodies = false;
        Piece* first_piece = &(L.begin()->datum);
        Piece* second_piece = NULL;
        Body* first_body = first_piece->B;
        
        for( PooledPieceList::Node* n = L.begin(); n!=L.end(); n = n->next )
        {
            if( n->datum.B != first_body )
            {
                second_piece = &(n->datum);
                two_bodies = true;
                break;
            }
        }
        
        if(!two_bodies) //if there's only one body in town, there can be no more collisions
            continue;
        
        
        // if out is true, we continue, leaving a record in piecePairs of the candidates for colliding pieces
        if( L.size() < 4 || mydepth <= 0 ) // i.e. if the list is small enough or if we've gone too deep, then continue
        {
            for( PooledPieceList::Node* nB = L.begin(); nB!=L.end(); nB = nB->next )
            for( PooledPieceList::Node* nA = L.begin(); nA!=nB; nA = nA->next )
            {
                Piece* A = nA->datum.mother;
                Piece* B = nB->datum.mother;
                
                if( A->B == B->B ) continue;
                
                if( A > B )
                {
                    Piece* buff = B;
                    B = A;
                    A = buff;
                }
                piecePairs.insert( pair<Piece*,Piece*>(A,B) );
            }
            
            continue;
        }
        
        
        for( PooledPieceList::Node* n = L.begin(); n!=L.end(); n = n->next )
        {
            Vec2 P(n->datum.P);
            Vec2 Q(n->datum.Q);
            double r = n->datum.r, arcA = n->datum.arcA, arcB = n->datum.arcB;
                
            switch(n->datum.tag)
            {   
                case kLinePiece:
                {
                    Vec2 PA( arcA*Q + (1.0-arcA)*P ),
                             PB( arcB*Q + (1.0-arcB)*P );
                    rect_left = min(rect_left, min(PA.x, PB.x));
                    rect_right = max(rect_right, max(PA.x, PB.x));
                    rect_bottom = min(rect_bottom, max(PA.y, PB.y));
                    rect_top = max(rect_top, min(PA.y, PB.y));
                }
                break;
                
                case kCirclePiece:
                {
                    Vec2 PA( P + r*Vec2(cos(arcA/r), sin(arcA/r)) ),
                             PB( P + r*Vec2(cos(arcB/r), sin(arcB/r)) );
                    
                    rect_left = min(rect_left, min(PA.x, PB.x));
                    rect_right = max(rect_right, max(PA.x, PB.x));
                    rect_bottom = min(rect_bottom, max(PA.y, PB.y));
                    rect_top = max(rect_top, min(PA.y, PB.y));
                    
                    if( arcA < arcB )
                    {
                        if( arcA <= r*PI && arcB >= r*PI ) rect_left = min(rect_right, P.x-r);
                        if( arcA <= r*0.5*PI && arcB >= r*0.5*PI ) rect_top = max(rect_top, P.y+r);
                        if( arcA <= r*1.5*PI && arcB >= r*1.5*PI ) rect_bottom = min(rect_bottom, P.y-r);
                    }
                    else
                    {
                        rect_right = max(rect_right, P.x+r);
                        if( arcB > r*PI || arcA < r*PI ) rect_left = min(rect_left, P.x-r);
                        if( arcB > r*0.5*PI || arcA < r*0.5*PI ) rect_top = max(rect_top, P.y+r);
                        if( arcB > r*1.5*PI || arcA < r*1.5*PI ) rect_bottom = min(rect_bottom, P.y-r);
                    }
                }
                break;
                
                default: break;
            }
        }
        
        
        
        PooledPieceList left(pool), right(pool);
        
        double  K1x, K1y, Dx, Dy, K2x, K2y;
        
        K1x = 0.5*(rect_right+rect_left);
        K1y = 0.5*(rect_top+rect_bottom);
        
        if( rect_right-rect_left > rect_top-rect_bottom ) { Dx = 0; Dy = 1; } else { Dx = 1; Dy = 0; }
        
        K2x = K1x + Dx;
        K2y = K1y + Dy;
        
        /*
        if( step==draw_step )
        {
            glColor3f(0,0.5,1);
            
            glBegin(GL_LINE_STRIP);
            glVertex3f(K1x - 1000*Dx, K1y - 1000*Dy,1);
            glVertex3f(K1x + 1000*Dx, K1y + 1000*Dy,1);
            glEnd();
        }
        */
        
        for( PooledPieceList::Node* n = L.begin(); n!=L.end(); n=n->next )
        {
            /***************CUT AND DIVVY INLINED****************/
            double Mx, My;
            double t,u;
            unsigned int Pleft, Qleft, Aleft;
            PooledPieceList *Aside, *Bside, *otherSide;
            
            p = n->datum;
            
            
            P.x = p.P.x;
            P.y = p.P.y;
            Q.x = p.Q.x;
            Q.y = p.Q.y;
            
            double r = n->datum.r;
            double  arcA = n->datum.arcA, 
                    arcB = n->datum.arcB;
            
            
            switch(n->datum.tag)
            {
                case kLinePiece:
                    Mx = (P.x-K1x);
                    My = (P.y-K1y);
                    Pleft = ( Dx*My > Dy*Mx );
                    Mx = (Q.x-K1x);
                    My = (Q.y-K1y);
                    Qleft = ( Dx*My > Dy*Mx );
                    
                    Aside = Pleft ? (&left) : (&right);
                    Bside = Pleft ? (&right) : (&left);
                    
                    if( Pleft ^ Qleft )
                    {
                        K1.set(K1x,K1y);
                        K2.set(K2x,K2y);
                        R = Line2d(K1,K2).intersectLine(Line2d(P,Q)); t = R[1];
                        
                        if( t<arcA ) { Bside->add(n->datum); continue; }
                        else if( t>arcB ) { Aside->add(n->datum); continue; }
                        else { p.arcA = arcA; p.arcB = t; Aside->add(p); p.arcA = t; p.arcB = arcB; Bside->add(p); continue; }
                    }
                    else { Aside->add(n->datum); continue; }
                break;
                
                
                case kCirclePiece:
                    /*first we want to check quickly if the piece is too far away from the separating line to be split*/
                    
                    Mx = P.x - K1x;
                    My = P.y - K1y;
                    double dxm = Dx*My - Dy*Mx;
                    if( dxm*dxm > r*r*(Dx*Dx + Dy*Dy) )
                    {
                        ((dxm > 0.0) ? (&left) : (&right)) -> add(n->datum);
                        continue;
                    }
                    
                    K1.set(K1x,K1y);
                    K2.set(K2x,K2y);
                    
                    trig_pair tp = fsin_and_cos(arcA/r);
                    Mx = r*tp.cos;
                    My = r*tp.sin;
                    
                    Mx += P.x - K1x;
                    My += P.y - K1y;
                    
                    Aleft = ( Dx*My - Dy*Mx > 0.0 );
                    
                    Aside = Aleft ? (&left) : (&right);
                    otherSide = Aleft ? (&right) : (&left);
                    
                    R = Circle(P,r).intersectLine(K1,K2);
                    t = R[0];
                    u = R[1];
                    
                    
                    D.set(Dx, Dy);
                    double theta = r*(angle(K1+t*D-P)), phi = r*(angle(K1+u*D-P));
                    if(theta<0) theta += 2.0*PI*r;
                    if(phi<0) phi += 2.0*PI*r;
                    
                    // make sure theta is less than or equal to phi:
                    if( phi < theta ){ double buff = theta; theta = phi; phi = buff; }
                                        
                    assert( !(arcB < arcA) );
                    
                    //int whichblock=-1;
                    
                    if( (arcB < theta) || (arcA > theta && arcB < phi) || (arcA > phi) ) { Aside->add(n->datum); continue; }
                    if( theta < arcA ) { p.arcA = arcA; p.arcB = phi; Aside->add(p); p.arcA = phi; p.arcB = arcB; otherSide->add(p); }
                    else if( !(phi > arcB) ) { p.arcA = arcA; p.arcB = theta; Aside->add(p); p.arcA = theta; p.arcB = phi; otherSide->add(p); p.arcA = phi; p.arcB = arcB; Aside->add(p); }
                    else { p.arcA = arcA; p.arcB = theta; Aside->add(p); p.arcA = theta; p.arcB = arcB; otherSide->add(p); }
                    
                break;
            }
            /***************END OF CUT AND DIVVY INLINED****************/
            
        }
        
        
        myStack[myStackPtr].L = right;
        myStack[myStackPtr].depth = mydepth-1;
        myStackPtr++;
        
        myStack[myStackPtr].L = left;
        myStack[myStackPtr].depth = mydepth-1;
        myStackPtr++;
    }
    
    
    /*
    if(draw_step > step)
    {
        printf( "no more\n" );
    }
    */
    
    free(myStack);
}



void Piece::draw() const
{
#if GL_ON
    switch(tag)
    {
        case kCirclePiece:
            glBegin(GL_LINE_STRIP);
            for( int i=0; i<=15; i++ )
            {
                double theta = arcA + (double(i))*((arcB-arcA)/15.0);
                theta/=r;
                glVertex3f( P.x + r*cos(theta), P.y + r*sin(theta), 1 );
            }
            glEnd();
        break;
        
        case kLinePiece:
            glBegin(GL_LINES);
                Vec2 A( P * (1.0-arcA) + Q * (arcA) ),
                        B( P * (1.0-arcB) + Q * (arcB) );
                glVertex3f( A.x, A.y, 1 );
                glVertex3f( B.x, B.y, 1 );
            glEnd();
        break;
    }
    
    glBegin(GL_LINE_STRIP);
    for( double theta=0.0; theta<=2.0*PI; theta+=PI/5.0 )
        glVertex3f( midpoint().x + 2*cos(theta), midpoint().y + 2*sin(theta), 1 );
    glEnd();
#endif
}



Vec3 compute_impulse_from_Vr(   Vec3 Vr,
                                    double m1, double I1, Vec3 r1,
                                    double m2, double I2, Vec3 r2,
                                    Vec3 N, double e )
{
    double vrn = Vr.dot(N);
    
    double  t1 = 1.0/m1,
            t2 = 1.0/m2,
            t3 = N.dot( r1.cross(N).cross(r1) )/I1,
            t4 = N.dot( r2.cross(N).cross(r2) )/I2;
    
    double j = -(1+e)*vrn / (t1+t2+t3+t4);
    
    return j*N;
}


Vec3 compute_impulse_with_friction( Vec3 v1, Vec3 w1, double m1, double I1, Vec3 r1,
                                        Vec3 v2, Vec3 w2, double m2, double I2, Vec3 r2,
                                        Vec3 N, double e, double u_s, double u_d )
{
    Vec3 Vr  = (w2.cross(r2) + v2) - (w1.cross(r1) + v1); //Vr = relative velocity at the point of impact
    Vec3 Vrn = projectOnto(Vr, N);
    Vec3 Vrt = Vr - Vrn; // the normal and tangential parts of Vr
    
    Vec3 newVrn, newVrt, newVr, j_stuck, j_slick;
    Mat3 K(KMatrix(m1, I1, r1) + KMatrix(m2, I2, r2));
    Mat3 Kinv(K.inverse());
    
    //first compute the impulse as if the objects are interacting statically, i.e. with 0 tangential relative velocity
    
    newVrn = -Vrn*e;
    newVrt.set(0,0,0); //(0 tangential)
    newVr  = newVrn + newVrt;
    
    j_stuck = Kinv*(newVr-Vr);
    
    //see if that impulse is in the 'friction cone', if it is, we're done
    
    Vec3 j_stuck_n = projectOnto(j_stuck, N);
    Vec3 j_stuck_t = j_stuck - j_stuck_n;
    if( j_stuck_t.magSquared() <= u_s*u_s*j_stuck.magSquared() )
        return j_stuck;
    
    //otherwise we have to compute a tangential frictional impulse:
    //first compute the unit tangent vector:
    Vec3 T(Vr - projectOnto(Vr, N));
    
    N = N/N.mag();

    //if the tangential relative motion is 0 anyway, we shouldn't actually get here,
    // but just in case, we just return the frictionless impulse
    if( T.mag()==0.0 )
    {
        double x = -(1+e)*Vr.dot(N) / N.dot(K*N);
        return x*N; //frictionless impulse
    }
    
    T = T/T.mag();
    
    //x is the magnitude of the normal component of j.
    //This is a formula for x obtained through some algebraic trickery:
    double x = -(1+e)*(Vr.dot(N)) / (N.dot(K*(N - u_d*T)));
    return x*(N-u_d*T);
}


Vec3 compute_impulse(   Vec3 v1, Vec3 w1, double m1, double I1, Vec3 r1,
                               Vec3 v2, Vec3 w2, double m2, double I2, Vec3 r2,
                               Vec3 N, double e )
{
    Vec3 Vr  = (w2.cross(r2) + v2) - (w1.cross(r1) + v1);
    Mat3 K(KMatrix(m1, I1, r1) + KMatrix(m2, I2, r2));
    
    N = N/N.mag();
    
    double x = -(1+e)*Vr.dot(N) / N.dot(K*N);
    
    return x*N;
}


Mat3 KMatrix(double m, double I, Vec3 r)
{
    double a = r.x, b = r.y, c = r.z;
    
    Mat3   M( b*b+c*c,    -a*b,    -a*c,
                     -a*b, a*a+c*c,    -b*c,
                     -a*c,    -b*c, a*a+b*b );
        
    return( Mat3()/m + M/I );
}


pair<Delta,Delta> Universe::bounce(const Collision& C)
{
    Body* P = C.P;
    Body* Q = C.Q;
    
    if( P->immutable() && Q->immutable() )
        return pair<Delta,Delta>();
    
    double e=0.0; //they use e for restitution
    
    switch(restitutionComputeType)
    {
        default:
        case kMinimum:
            e = min( P->restitution, Q->restitution ); //the min is appaently the standard number to use
            break;
        
        case kMaximum:
            e = max( P->restitution, Q->restitution );
            break;
        
        case kAverage:
            e = 0.5*(P->restitution + Q->restitution);
            break;
    }
    
    if( forceRestitution )
        e = forcedRestitution;
    
    
    double u_s, u_d;
    
    switch(frictionComputeType)
    {
        case kMinimum:
            u_s = min( P->staticFriction, Q->staticFriction );
            u_d = min( P->dynamicFriction, Q->dynamicFriction );
            break;
        
        default: //for friction, it's the max (according to lore)
        case kMaximum:
            u_s = max( P->staticFriction, Q->staticFriction );
            u_d = max( P->dynamicFriction, Q->dynamicFriction );
            break;
        
        case kAverage:
            u_s = 0.5*(P->staticFriction + Q->staticFriction);
            u_d = 0.5*(P->dynamicFriction + Q->dynamicFriction);
            break;
    }
    u_s = max(u_d,u_s); //u_s should always be bigger than u_d no matter what.
    
    
    Vec3 N( C.normalOfImpact, 0.0 );
    N/=N.mag();
    
    double t = C.timeOfImpact;
    
    Vec2 	R1_2d( C.pointOfImpact - (P->position + t*P->velocity) ),
                R2_2d( C.pointOfImpact - (Q->position + t*Q->velocity) );
    
    Vec3 R1( R1_2d, 0.0 ), R2( R2_2d, 0.0 );
    
    Vec3 V1(P->velocity, 0.0);
    Vec3 V2(Q->velocity, 0.0);
    Vec3 w1(0.0, 0.0, P->omega);
    Vec3 w2(0.0, 0.0, Q->omega);
    
    double  m1 = P->mass,
            m2 = Q->mass,
            I1 = P->rotationalInertia,
            I2 = Q->rotationalInertia;
     
    //the coeficient of restitution gets multiplied by a loss factor to leak a little extra energy
    //this massages away a bug which causes things to bounce higher and higher
    double vr = ((w2.cross(R2) + V2) - (w1.cross(R1) + V1)).mag();
    
    double smallvr = 0.1; // this number smallvr in here should be a variable in Universe or a constant or something
    if(vr < smallvr)
        e = 0.0;
    else
        e *= (1.0 - smallvr/vr);
    
    Vec3 impulse = compute_impulse_with_friction( 	V1, w1, m1, I1, R1, 
                                                        V2, w2, m2, I2, R2,
                                                        N, e, u_s, u_d  );
    
    impulse += randVec3(impulseJitter); //very experimental jitter (again I fished around for a good number)
    if(nudgeForceOn)
        impulse+=nudgeForceAmmount*N;
    
    Vec2 impulse2d(impulse.x, impulse.y);
    
    pair<Delta,Delta> deltaPQ( Delta(t,-impulse2d/m1, -(R1.cross(impulse)/I1).z ),
                               Delta(t, impulse2d/m2,  (R2.cross(impulse)/I2).z ) );
    
    return deltaPQ;
}





void Universe::drawAllCollisions()
{
#if GL_ON
    glColor3f( 0.0, 1.0, 0.0 );
    drawCollisionSet( findAllCollisions(), 30 );
#endif
}

void Universe::drawAllCollisionsBinary()
{
#if GL_ON
    glColor3f( 1.0, 0.0, 0.0 );
    drawCollisionSet( findAllCollisionsBinary(), 15 );
#endif
}

void Universe::drawCollisionSet(const set<Collision>& S, double arrowLength)
{
#if GL_ON
    for( set<Collision>::iterator itr = S.begin(); itr!=S.end(); itr++ )
    {
        glBegin(GL_LINES);
        
        Vec2 A((*itr).pointOfImpact), B;
        B = A + (*itr).normalOfImpact*arrowLength;
        
        glVertex3f( A.x, A.y, 1 );
        glVertex3f( B.x, B.y, 1 );
        
        glEnd();
        
        
        glBegin(GL_LINE_STRIP);
        
        double theta = 0;
        for( int i=0; i<=5; i++ )
        {
            theta += 4*PI/5.0;
            Vec2 V = (*itr).pointOfImpact + 4.0*Vec2(cos(theta), sin(theta));
            glVertex3f( V.x, V.y, 1 );
        }
        
        glEnd();
    }
#endif
}


void Universe::activateThreading()
{
    threading = true;
    pthread_mutex_t Z = PTHREAD_MUTEX_INITIALIZER;
    bodyMutex = Z;
    bailMutex = Z;
}


} //end namespace
