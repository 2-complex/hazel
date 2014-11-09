
#include "spriteenvironment.h"
#include "graphics.h"

class QuadEnvironment : public SpriteEnvironment
{
    Buffer* buffer;
    IndexBuffer* indexBuffer;
    Geometry geometry;
    Effect effect;
    Assumption material;
    mutable Assumption camera;
    Texture2D texture;
    Shape shape;

    void init();
    void draw() const;
};

void QuadEnvironment::init()
{
    float vertexArray[] =
    {
        -1, -1, 0,
         1, -1, 0,
         1,  1, 0,
        -1,  1, 0,
    };

    int indexArray[] =
    {
        0,2,1,
        0,2,3
    };

    buffer = new Buffer(vertexArray, 12);
    indexBuffer = new IndexBuffer(indexArray, 12);

    geometry["position"] = Field(buffer, 3, 3, 0);
    geometry.indices = indexBuffer;

    Bitmap bitmap;
    bank.initBitmapWithPath(&bitmap, "tiles.png");
    texture.initWithBitmap(bitmap);

    effect.vertexCode =
        "\n"
        "varying vec3 v_position;\n"
        "\n"
        "attribute vec3 position;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    v_position = position.xyz;\n"
        "    gl_Position = vec4(position, 1.0);\n"
        "}\n";

    effect.fragmentCode =
        "\n"
        "uniform sampler2D sampler;\n"
        "varying vec3 v_position;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    gl_FragColor = texture2D(sampler, v_position.xy);\n"
        "}\n";

    effect.compile();

    material.effect = &effect;

    shape.geometry = &geometry;
    shape.assumptions.push_back(&material);
}

void QuadEnvironment::draw() const
{
    shape.draw();
}

int main()
{
    QuadEnvironment e;
    e.mainLoop();

    return 0;
}


