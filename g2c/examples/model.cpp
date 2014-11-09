
#include "panenvironment.h"
#include "graphics.h"

#include "macbank.h"

#include <stdio.h>

using namespace g2c;

class MyPanEnvironment : public PanEnvironment {
public:
	char* filename;
	
	Buffer* buffer;
	IndexBuffer* indexBuffer;
	Field* position;
	
	Geometry geometry;
	Shape quad;
	Effect effect;
	Assumption material;

private:
	void init();
	void draw() const;
};

void MyPanEnvironment::init()
{
	double v[] = {-1,-1,0,1,-1,0,1,1,0,-1,1,0};
	buffer = new Buffer(v, sizeof(v)/sizeof(double));
	
	int i[] = {0,1,2,0,2,3};
	indexBuffer = new IndexBuffer(i, sizeof(i)/sizeof(int));
	
	position = new Field(buffer, 3);
	
	geometry.indices = indexBuffer;
	
	geometry["position"] = *position;
	
	effect.vertexCode =
        "attribute vec3 position;\n"
        "varying vec3 v_position;\n"
        "\n"
        "void main()\n"
        "{\n"
        "  v_position = position;\n"
        "  gl_Position = vec4(position, 1.0);\n"
        "}\n";
    
    effect.fragmentCode =
        "varying vec3 v_position;\n"
        "\n"
        "void main()\n"
        "{\n"
        " gl_FragColor = vec4(v_position.xy, 1.0, 1.0);\n"
        "}\n";
    
    effect.compile();
    
    material.effect = &effect;
	quad.geometry = &geometry;
	
	quad.assumptions.push_back(&material);
}

void MyPanEnvironment::draw() const
{
	quad.draw();
}

int main(int argc, char** args)
{
	MyPanEnvironment e;
	
	e.filename = (char*)"box.model";
	if( argc > 1 )
		e.filename = args[1];
	
	e.mainLoop();
	
	return 0;
}

