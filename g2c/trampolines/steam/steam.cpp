
#include "steam.h"

void SteamApp::setBank(Bank* bank)
{
    this->bank = bank;
}

void SteamApp::reshape(int width, int height)
{
    projection = perspective(
        40.0,
        1.0 * width / height,
        0.1 * cameraRadius,
        1000 * cameraRadius );

    modelView = g2c::lookAt(
        Vec3(1, 5, 1),
        Vec3(0, 0, 0),
        Vec3(0, 0, 1));

    glViewport(0,0,width,height);
}

void SteamApp::init()
{
    float vertexArray[] =
    {
         -1.0, -1.0, 0.0,
          1.0, -1.0, 0.0,
          1.0,  1.0, 0.0,
         -1.0,  1.0, 0.0,
    };

    int indexArray[] =
    {
        0,1,2,
        0,2,3,
    };

    buffer.set(vertexArray, sizeof(vertexArray) / sizeof(float));
    indexBuffer.set(indexArray, sizeof(indexArray) / sizeof(int));

    geometry["position"] = Field(&buffer, 3, 3, 0);
    geometry.indices = &indexBuffer;

    effect.vertexCode =
        "\n"
        "uniform mat4 modelView;\n"
        "uniform mat4 projection;\n"
        "uniform vec3 center;\n"
        "uniform float radius;\n"
        "\n"
        "attribute vec3 position;\n"
        "varying vec3 v_position;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    gl_Position = projection * (modelView * vec4(center, 1.0) + vec4(radius * position, 0.0) );\n"
        "    v_position = position;\n"
        "}\n";

    effect.fragmentCode =
        "\n"
        "uniform vec4 color;\n"
        "varying vec3 v_position;\n"
        "uniform float radius;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    gl_FragColor = 0.3 * color / (radius*radius) / (1.0 + 5.0*dot(v_position.xy, v_position.xy));\n"
        "}\n";

    effect.compile();

    material.effect = &effect;

    shape.geometry = &geometry;
    shape.assumptions.push_back(&camera);
    shape.assumptions.push_back(&particle);
    shape.assumptions.push_back(&material);

    material["color"] = Vec4(1.0, 1.0, 1.1, 1.0);
    last_t = 0;

    cameraRadius = 4.0;
}

void SteamApp::step(double t)
{
    if ( t - last_t > 1.0 / 30.0 )
    {
        for( vector<Particle>::iterator itr = particles.begin();
            itr != particles.end();
            itr++ )
        {
            itr->center += itr->deltaCenter;
            itr->radius += itr->deltaRadius;

            if ( itr->radius > 8 )
            {
                itr->center = Vec3(0,0,-10);
                itr->radius = .5;
            }
        }

        if ( particles.size() < 80 )
        {
            particles.push_back(
                Particle( Vec3(0.0, 0.0, -10.0), .5 ) );
        }

        last_t = t;
    }
}

void SteamApp::draw() const
{
    camera["modelView"] = modelView;
    camera["projection"] = projection;

    glDisable(GL_DEPTH_TEST);
    glBlendFunc(GL_ONE, GL_ONE);

    for( vector<Particle>::const_iterator itr = particles.begin();
        itr != particles.end();
        itr++ )
    {
        particle["center"] = itr->center;
        particle["radius"] = itr->radius;
        shape.draw();
    }
}

