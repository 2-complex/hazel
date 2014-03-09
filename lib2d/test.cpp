
#include "math2d.h"
#include "phys2d.h"


using namespace std;
using namespace lib2d;

int main( int argc, char** args )
{
    printf( "sanity: check!\n" );
    
    Polygon P;
    P.add(0,0).add(1,0).add(1,1).add(0.5,2);
    printf( "%f\n", P.area() );
    
    return 0;
}

