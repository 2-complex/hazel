
#include "sprites.h"
#include "graphics.h"

namespace g2c
{
    /*! An implementation of Renderer to draw meshes using g2c's graphics library.

        Set Mesh::renderer to an instance of RendererG to and call init().  Then all meshes
        will draw using it.*/
    class RendererG : public Renderer {
    public:
        RendererG();
        virtual ~RendererG();

    private:
	Buffer quadBuffer;
	IndexBuffer quadIndexBuffer;

	mutable Buffer polygonBuffer;
        mutable IndexBuffer polygonIndexBuffer;

	Effect effect;

        Geometry quadGeometry;
	Shape quadShape;

	Geometry polygonGeometry;
	Shape polygonShape;

	mutable Assumption assumption;

        Texture2D* defaultTexture;
    public:
        virtual void init();

    protected:
        bool initialized;
        virtual void drawMesh(
            const Mesh* m,
	    const Mat4& matrix,
	    const Mat3& texMatrix,
	    const Color& color,
	    const Texture* texture) const;
    };
}

