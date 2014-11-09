
#include "serializable.h"

using namespace g2c;

class Vec : public Serializable {
public:
	Vec();
	
	DoubleProperty x;
	DoubleProperty y;
};

Vec::Vec()
{
	addProperty("x", x);
	addProperty("y", y);
}

class Apple : public Serializable {
public:
	Apple();
	
	IntProperty i;
	BoolProperty b;
	DoubleProperty x;
	StringProperty s;
	Vec v;
	VectorProperty<IntProperty> is;
	PointerVectorProperty<Vec*> vecs;
};

Apple::Apple()
{
	type = "Apple";
	addProperty("i", i);
	addProperty("b", b);
	addProperty("x", x);
	addProperty("s", s);
	addProperty("v", v);
	addProperty("is", is);
	addProperty("vecs", vecs);
}

int main()
{
	Apple a;
	
	a.i = 37;
	a.b = true;
	a.x = 12.345;
	a.s = "My mother loves wordgames.";
	a.is.push_back(3);
	a.is.push_back(4);
	a.is.push_back(5);
	
	Vec u,v,w;
	
	u.x = 30;
	u.y = 40;
	
	v.x = 50;
	v.y = 70;

	w.x = 110;
	w.y = 120;
	
	a.vecs.push_back(&u);
	a.vecs.push_back(&v);
	a.vecs.push_back(&w);
	
	printf( "%s\n", a.serialize().c_str() );
	
	return 0;
}

