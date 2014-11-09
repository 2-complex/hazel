
#include "rendererg.h"

namespace g2c
{

RendererG::RendererG() :
    initialized(false),
    defaultTexture(NULL)
{
    
}

RendererG::~RendererG()
{
    if( defaultTexture )
        delete defaultTexture;
}

void RendererG::init()
{
    quadBuffer = quad->positions;
    quadIndexBuffer = quad->indices;

    const char* vertexCode = "\n"
        "    attribute vec3 position;\n"
        "    uniform mat4 matrix;\n"
        "    uniform mat3 texMatrix;\n"
        "    varying vec2 v_texcoord;\n"
        "    void main() {\n"
        "        vec3 t = texMatrix * vec3(position.xy, 1.0);\n"
        "        v_texcoord = t.xy / t.z;\n"
        "        gl_Position = matrix * vec4(position, 1.0);\n"
        "    }\n";

    const char* fragmentCode = "\n"
        "#ifdef GL_ES\nprecision highp float;\n#endif\n"
        "    uniform vec4 color;\n"
        "    uniform sampler2D texture;\n"
        "    varying vec2 v_texcoord;\n"
        "    void main() {\n"
        "        gl_FragColor = color * texture2D(texture, v_texcoord);\n"
        "    }\n";

    effect.vertexCode = vertexCode;
    effect.fragmentCode = fragmentCode;
    effect.compile();
    assumption.effect = &effect;

    quadGeometry["position"] = Field(&quadBuffer, 3, 3, 0);
    quadGeometry.indices = &quadIndexBuffer;
    quadShape.geometry = &quadGeometry;

    polygonGeometry["position"] = Field(&polygonBuffer, 3, 3, 0);
    polygonGeometry.indices = &polygonIndexBuffer;
    polygonShape.geometry = &polygonGeometry;

    defaultTexture = new Texture2D;
    GLubyte data[] = {255, 255, 255, 255};
    defaultTexture->initWithImageData(data, 1, 1, 32);

    quadShape.assumptions.push_back(&assumption);
    polygonShape.assumptions.push_back(&assumption);

    initialized = true;
}

void RendererG::drawMesh(
    const Mesh* mesh,
    const Mat4& matrix,
    const Mat3& texMatrix,
    const Color& color,
    const Texture* texture) const
{
    if( !texture )
        texture = defaultTexture;

    assumption["matrix"] = projection * matrix;
    assumption["texMatrix"] = texMatrix;
    assumption["color"] = color;
    assumption["texture"] = texture;

    if( mesh )
    {
	polygonBuffer = mesh->positions;
        polygonIndexBuffer = mesh->indices;
        polygonShape.draw();
    }
    else
    {
	quadShape.draw();
    }
}

}

