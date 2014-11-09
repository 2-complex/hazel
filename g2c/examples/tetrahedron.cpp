
#include "panenvironment.h"
#include "graphics.h"

using namespace std;

class TetrahedronEnvironment : public PanEnvironment
{
    Buffer buffer;
    IndexBuffer indexBuffer;
    Geometry geometry;
    Effect effect;
    Assumption material;
    mutable Assumption camera;
    Shape shape;

    void init();
    void draw() const;  
};

void TetrahedronEnvironment::init()
{
    float vertexArray[] =
    {
         1.0,  0.0,     -0.25 * 1.41421,
        -0.5,  0.86603, -0.25 * 1.41421,
        -0.5, -0.86603, -0.25 * 1.41421,
         0.0,  0.0,      0.75 * 1.41421
    };

    int indexArray[] =
    {
        0,2,1,
        0,1,3,
        1,2,3,
        2,0,3
    };

    buffer.set(vertexArray, sizeof(vertexArray) / sizeof(float));
    indexBuffer.set(indexArray, sizeof(indexArray) / sizeof(int));

    geometry["position"] = Field(&buffer, 3, 3, 0);
    geometry.indices = &indexBuffer;

    effect.vertexCode = 
        "\n"
        "uniform mat4 modelView;\n"
        "uniform mat4 projection;\n"
        "\n"
        "attribute vec3 position;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    gl_Position = projection * modelView * vec4(position, 1.0);\n"
        "}\n";

    effect.fragmentCode =
        "\n"
        "uniform vec4 color;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    gl_FragColor = color;\n"
        "}\n";
    
    effect.compile();

    material.effect = &effect;

    shape.geometry = &geometry;
    shape.assumptions.push_back(&camera);
    shape.assumptions.push_back(&material);

    material["color"] = Vec4(1.0, 0.0, 0.0, 1.0);
}

void TetrahedronEnvironment::draw() const
{
    camera["modelView"] = getModelView();
    camera["projection"] = getProjection();

    shape.draw();
}

int main()
{
    TetrahedronEnvironment e;
    e.mainLoop();

    return 0;
}


