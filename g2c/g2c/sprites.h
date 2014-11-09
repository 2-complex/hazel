
/*
  g2c Copyright (C) 2012 2-Complex

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/


#ifndef _SPRITES_
#define _SPRITES_

#include "listener.h"
#include "parse.h"
#include "serializable.h"
#include "bank.h"
#include "texture.h"
#include "sound.h"

#include "util.h"
#include "lin/lin.h"

#include "opengl.h"

#include <vector>
#include <string>
#include <set>
#include <map>


namespace g2c {
    class Color;
    class Node;
    class Animator;
    class Animation;
    class Renderer;
    class Sampler;
    class Sprite;
    class Button;
    class ButtonHandler;
    class Font;
    class Actor;
    class Layer;
    class World;
    class Mesh;
    class Polygon;


    class Mat4Property
        : public Mat4
        , public Serializable
    {
    public:
        Mat4Property();
        Mat4Property(const Mat4& m);

        virtual std::string serialize(std::string indent) const;
        void initWithParseNode(const parse::Node* n);
    };

    class Vec2Property
        : public Vec2
        , public Serializable
    {
    public:
        Vec2Property();
        Vec2Property(const Vec2& v);

        virtual std::string serialize(std::string indent) const;
        void initWithParseNode(const parse::Node* n);
    };

    class Vec3Property
        : public Vec3
        , public Serializable
    {
    public:
        Vec3Property();
        Vec3Property(const Vec3& v);

        virtual std::string serialize(std::string indent) const;
        void initWithParseNode(const parse::Node* n);
    };

    class Color : public Vec4
    {
    public:
        Color() : Vec4(), r(x), g(y), b(z), a(w) { r = g = b = a = 1.0; }
        Color(double r, double g, double b, double a) : Vec4(r, g, b, a), r(x), g(y), b(z), a(w) {}
        Color(const Color& c) : Vec4(c), r(x), g(y), b(z), a(w) {}
        Color(const Vec4& c) : Vec4(c), r(x), g(y), b(z), a(w) {}
        inline Color& operator= (const Color& v) {x=v.x; y=v.y; z=v.z; w=v.w; return *this;}

        double &r, &g, &b, &a;
    };

    class ColorProperty
        : public Color
        , public Serializable
    {
    public:
        ColorProperty();
        ColorProperty(const Color& c);

        inline ColorProperty& operator= (const Color& v) {x=v.x; y=v.y; z=v.z; w=v.w; return *this;}

        virtual std::string serialize(std::string indent) const;
        void initWithParseNode(const parse::Node* n);
    };

    /*! Node is a node in a transform tree.  A Node contians a vector of children and a pointer
        to a parent.  Use the function add() to add a Node to another Node as a child.  Use remove()
        to remove child nodes.  Node::draw() iterates through the children and calls draw on each
        one.  Subclasses of Node implement drawInTree(worldMatrix, worldColor) to draw using the
        given matrix and color. */
    class Node
        : public Serializable
        , public Listener
    {
    public:
        Node();
        virtual ~Node();

        BoolProperty visible;
        ColorProperty color;

        virtual Mat4 getMatrix() const;
        virtual Color getColor() const;

        virtual Mat4 getWorldMatrix() const;
        virtual Color getWorldColor() const;

        Node* parent;
        PointerVectorProperty<Node*> children;

        Node* findChild(const std::string& name) const;
        void add(Node* t);
        void addAtIndex(Node* t, int index);
        int indexOf(const Node* t) const;

        void remove(Node* t);
        void removeAndDelete(Node* t);
        void clear();

        virtual void removeSampler(const Sampler* sampler);

        void clearChildren();

        void draw() const;
        virtual void drawInTree(const Mat4& worldMatrix, const Color& worldColor) const;

        virtual Actor* actorInClick(const Mat4& worldMatrix, const Vec2& C);

        virtual void handleChild(const parse::Node* n);

        virtual bool vectorInside(const Mat4& worldMatrix, const Vec2& V) const;

        // Methods of Listener.
        virtual void keyboard(unsigned char inkey);
        virtual bool mouseDown(const Vec2& C);
        virtual void mouseDragged(const Vec2& C);
        virtual void mouseUp(const Vec2& C);

    protected:
        std::vector<Serializable*> deleteMe;

        virtual bool mouseDown(const Mat4& worldMatrix, const Vec2& C);
        virtual void mouseDragged(const Mat4& worldMatrix, const Vec2& C);
        virtual void mouseUp(const Mat4& worldMatrix, const Vec2& C);

        bool tookMouseDown;
        void clearTookMouseDown();
    };

    /*! Mesh represents a collection of triangles or line segments in 2D.  To draw a mesh, populate positions and vertices,
        then use a Renderer implementation's drawMesh() function. */
    class Mesh
    {
    public:
        Mesh();
        virtual ~Mesh();

        enum ElementType
        {
            kTriangles,
            kLines
        };

        static Renderer* renderer;

        void resize(int inNumberOfVertices, int inNumberOfElements);

        ElementType elementType;

        std::vector<float> positions;
        std::vector<unsigned short> indices;

        int numberOfVertices() const;
        int numberOfElements() const;
    };

    /*! Renderer is an abstract base-class whose virtual methods define how a Mesh shall be drawn.
        Subclasses of Renderer represent a scheme for drawing a mesh in a particular graphics
        library.

        To implement a renderer, implement init() and drawMesh().  Whichever renderer
        Sprite::renderer is set to gets used to draw all sprite graphics.*/
    class Renderer
    {
    public:
        Renderer();
        virtual ~Renderer();

        Mat4 projection;

    protected:
        mutable GLfloat fv[16];
        Mesh* quad;

    public:
        /*! Overridden to initialize the renderer.*/
        virtual void init() = 0;

        /*! Overridden to draw a Mesh object.  If mesh is NULL, drawMesh must still draw a default
            unit square.   */
        virtual void drawMesh(
            const Mesh* mesh,
            const Mat4& matrix,
            const Mat3& texMatrix,
            const Color& color,
            const Texture* texture) const = 0;
    };

    /*! An implementation of Renderer to draw meshes using OpenGL calls from the OpenGL ES 1
        collection.  The draw function in this will work in desktop OpenGL or in OpenGL ES 1.

        Set Mesh::renderer to an instance of RendererGL1 to and call init().  Then all meshes
        will draw using it.*/
    class RendererGL1 : public Renderer
    {
    public:
        RendererGL1();
        ~RendererGL1();

        virtual void init();
        virtual void drawMesh(
            const Mesh* mesh,
            const Mat4& matrix,
            const Mat3& texMatrix,
            const Color& color,
            const Texture* texture) const;
    };

    /*! An implementation of Renderer to draw meshes using OpenGL calls from the OpenGL ES 2
        collection.  The draw function in this will work in desktop OpenGL or in OpenGL ES 2.

        Set Mesh::renderer to an instance of RendererGL2 to and call init().  Then all meshes
        will draw using it.*/
    class RendererGL2 : public Renderer {
    public:
        RendererGL2();
        virtual ~RendererGL2();

    private:
        GLuint buffer;
        GLuint indexBuffer;
        GLuint program;
        GLuint positionLocation;
        GLuint vertexShader;
        GLuint fragmentShader;
        GLuint matrixLocation;
        GLuint SpriteLocation;
        GLuint colorLocation;
        GLuint textureLocation;
        GLuint texMatrixLocation;
        GLuint polygonBuffer;
        GLuint polygonIndexBuffer;

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


    class Polygon : public Node {
    public:
        Polygon();
        virtual ~Polygon();

        enum DrawType {
            kSolid,
            kOutline,
        };

        Mat4Property matrix;

        Mat4 getMatrix() const;

        Polygon(const Polygon& P);
        Polygon& operator=(const Polygon& P);

        Polygon& add( const Vec2& V );
        Polygon& add( double x, double y );

        Polygon& operator += (const Vec2& V);
        Polygon& operator -= (const Vec2& V);
        Polygon& operator*= (double k);
        Polygon& operator/= (double k);
        Polygon operator + (const Vec2& V) const;
        Polygon operator - (const Vec2& V) const;
        Polygon operator* (double k) const;
        Polygon operator/ (double k) const;
        friend class Polygon operator * (const Mat4&, const Polygon& P);
        inline friend class Polygon operator * (double k, const Polygon& P) {return P*k;}
        inline friend class Polygon operator + (const Vec2& V, const Polygon& P) {return P+V;}
        void rotate(double theta);

        std::vector<Vec2>::size_type size() const;
        bool empty() const;

        bool vectorInside( const Mat4& worldMatrix, const Vec2& V ) const;

        void reverse();
        void setVertices(const std::vector<Vec2>& p);
        std::vector<Vec2> getVertices() const;
        std::vector<int> getTriangleIndices() const;

        DrawType getDrawType() const;
        void setDrawType(DrawType inDrawType);

        virtual void handleChild(const parse::Node* n);
        virtual std::string serializeElements(std::string indent = "") const;

        VectorProperty<Vec2Property> vertices;

        virtual void drawInTree(const Mat4& worldMatrix, const Color& worldColor) const;

    private:
        mutable Mesh* mesh;
        DrawType drawType;
        mutable bool valid;

        void update() const;
        void triangulate() const;
    };

    /*! An Actor represents 2D graphics on the screen, typically a sprite at a particular location
        with a current frame of animation.  Several actors might use the same Sampler object to
        draw.  For instance, a game with a lot of visually identical bullets on the screen would
        draw several Actors all pointing to the same bullet Sprite.

        An Actor has a pointer to a Sampler and also a Mesh.  Actor::draw() draws the mesh using
        the sampler to select a matrix in a texture.  If no Mesh is specified, a default square
        mesh is used.

        Actors can be positioned and scaled using x,y, and k, also a frame of animation that is
        current.  The frame gets used to compute texture coordiates when drawing the mesh. */
    class Actor : public Node
    {
    public:
        Actor();

        /*! Initializes the Actor with given Sprite object. */
        explicit Actor(Sampler* inSampler);
        virtual ~Actor();

        Sampler* sampler;
        Sampler*& sprite;
        Mesh* mesh;

        /*! The x coordinate where to draw on the screen.  Modifying x changes position. */
        double& x;

        /*! The x coordinate where to draw on the screen.  Modifying y changes position.*/
        double& y;

        /*! A scaling factor. */
        DoubleProperty k;

        /*! The index of the current frame of animation to draw. */
        IntProperty frame;

        /*! The 2D coordinates where to draw on the screen. */
        Vec2Property position;

        /*! An angle measured in radians to rotate the actor. */
        DoubleProperty rotation;

        virtual Mat4 getMatrix() const;

        Polygon polygon;

        virtual void handleChild(const parse::Node* n);
        virtual Polygon collisionPolygon() const;
        bool vectorInside(const Mat4& worldMatrix, const Vec2& C) const;

        Actor* actorInClick(const Mat4& worldMatrix, const Vec2& C);
        virtual void removeSampler(const Sampler* sampler);

        /*! Draws mesh using sampler as a texture.  If mesh is NULL, uses a default unit square Mesh.*/
        virtual void drawInTree(const Mat4& worldMatrix, const Color& worldColor) const;

    private:
        void init();

        StringProperty samplerName;
    };

    /*! Actor for drawing text from a string and a Font object.  Text::draw draws a quad for each
        character in its member string s, sampling the texture represented by font to get
        characters. */
    class Text : public Actor {
    public:
        Text();
        explicit Text(Font* infont);

        /*! A string either "left", "right" or "center" describing how the drawn string eminates
            from the coordinates of this Actor. */
        std::string justification;

        /*! A standard c++ string, this is the text that gets drawn. */
        std::string s;

        /*! A font object is a Sampler whose frames correspond to the characters to be drawn. */
        Font* font;

        std::string fontName;

        bool drawPipe;

        virtual std::string serializeElements(std::string indent) const;
        virtual void handleChild(const parse::Node* n);

        virtual Polygon collisionPolygon() const;

        // Methods of listener.
        void keyboard(unsigned char inkey);

        /*! Draws an image of the c++ string s using the characters in font. */
        virtual void drawInTree(const Mat4& worldMatrix, const Color& worldColor) const;
    };

    class Integer : public Text {
    public:
        Integer();
        Integer(Font* infont, int* inptr);

        int* ptr;

        /*! Draws an image of the c++ string s using the characters in font. */
        virtual void drawInTree(const Mat4& worldMatrix, const Color& worldColor) const;
    };

    /*! A Sampler is a texture together with a method by which a texture is sampled to draw
        a quad on the sreen.  Override getTexMatrix() to return a matrix representing the
        transformation of a unit quad [(0,0), (1,1)] to the quad in the texture to be sampled
        based on the frame number. */
    class Sampler : public Texture2D
    {
    public:
        Sampler();
        virtual ~Sampler();

        StringProperty file;

        virtual Mat4 getOffsetMatrix(int frame = 0, double x = 0.0, double y = 0.0, double k = 1.0) const;
        virtual Mat3 getTexMatrix(int frame = 0) const = 0;

        virtual double getWidth(int frame = 0) const = 0;
        virtual double getHeight(int frame = 0) const = 0;
        virtual bool getCenter(int frame = 0) const = 0;

        virtual Polygon getCollisionPolygon(int frame = 0) const = 0;
    };


    /*! Represents an axis aligned rectangle.  Mainly for use by Atlas which uses it to sample
        a texture from various do draw sprite graphics.*/
    class Rectangle : public Serializable
    {
    public:
        Rectangle();
        Rectangle( double x0, double y0, double x1, double y1 );
        virtual ~Rectangle();

        double x0;
        double y0;
        double x1;
        double y1;

    public:
        std::string serialize(std::string indent = "") const;
        void initWithParseNode(const parse::Node* n);
    };

    /*! A Sprite represnts a texture on the GPU whose image is a rectangular array of animation
        frames.  For instance a chracter in a side-scroller could use a Sprite to hold all the
        possible poses.

        Sprite can be initialized by using the functions of its parent class Texture2D to load in a
        Bitmap and then setting numberOfRows and numberOfColumns.

        The class Actor is made to draw a Sprite on the screen with specific coordinates and current
        animation frames. */
    class Sprite : public Sampler
    {
    public:
        Sprite();
        virtual ~Sprite();

        IntProperty numberOfColumns;
        IntProperty numberOfRows;

        BoolProperty center;
        BoolProperty flipRows;

        Polygon polygon;

        static bool drawLines;

        virtual Mat3 getTexMatrix(int frame = 0) const;
        virtual double getWidth(int frame = 0) const;
        virtual double getHeight(int frame = 0) const;
        virtual bool getCenter(int frame = 0) const;
        virtual Polygon getCollisionPolygon(int frame = 0) const;

    protected:
        virtual std::string serializeElements(std::string indent = "") const;
        virtual void handleChild(const parse::Node* n);
    };


    /*! An Atlas is a texture which is subdivided into rectangular images to be sampled by an
        Actor.  Here, a frame of animation references a rectangle in the list.*/
    class Atlas : public Sampler
    {
    public:
        Atlas();
        virtual ~Atlas();

        BoolProperty center;
        VectorProperty<Rectangle> rectangles;
        VectorProperty<Polygon> polygons;

        virtual Mat3 getTexMatrix(int frame = 0) const;
        virtual double getWidth(int frame = 0) const;
        virtual double getHeight(int frame = 0) const;
        virtual bool getCenter(int frame = 0) const;
        virtual Polygon getCollisionPolygon(int frame = 0) const;
    };


    /*! Button is a type of Actor that is clickable, it is meant to draw as a textured quad
        using a Sampler object just like its parent, except that it has its own implementations
        of mouseDown, mouseDragged and mouseUp which handle the animation of the button getting
        pushed.

        To use a Button, instantiate one, set the button's sampler, set the integer baseFrame
        to the index of the frame of the sampler for the button in resting state.  The next frame
        after that is assumed to be for the depressed state.

        To implement a behaviour for the button when pressed, subclass ButtonHandler, override
        click() function, and then set the button's handler element to an instance of that
        subclass.*/
    class Button : public Actor {
    public:
        Button();
        explicit Button(Sampler* insampler);
        Button(Sampler* insampler, int inBaseFrame);

        int baseFrame;
        bool enabled;

        virtual void drawInTree(const Mat4& worldMatrix, const Color& worldColor) const;

        virtual void handleChild(const parse::Node* n);
        std::string serializeElements(std::string indent) const;

        ButtonHandler* handler;

    protected:
        virtual bool mouseDown(const Mat4& worldMatrix, const Vec2& C);
        virtual void mouseDragged(const Mat4& worldMatrix, const Vec2& C);
        virtual void mouseUp(const Mat4& worldMatrix, const Vec2& C);

    private:
        bool depressed;
    };

    /*! Abstract helper class for Button the purpose of which is to hold the virtual function
        click().  Subclass ButtonHandler and override click, then set a Button's handler element
        to an instnace of the subclass.  click() will be called when the button is clicked. */
    class ButtonHandler {
    public:
        ButtonHandler();
        virtual ~ButtonHandler();

        virtual void down(Button* button);
        virtual void up(Button* button);

        virtual void click(Button* button) = 0;
    };

    /*! Font is a type of Sampler which contains images of characters.  It is is drawn using a
        Text object. */
    class Font : public Sprite {
    public:
        Font();

        std::vector<double> widths;
        std::vector<double> lefts;
        double lineHeight;
        double lineBottom;
        double widthScale;
        double spacing;
        char baseChar;

        double charWidth(char c) const;
        double charLeft(char c) const;
        double lineWidth(double k, const char* s, int startIndex) const;
        std::vector<std::pair<int, int> > lineIndices(const char* s) const;
        void drawString(
            const Mat4& M,
            const Color& color,
            double x,
            double y,
            double k,
            const char* s,
            const std::string& justification = "left") const;
        Polygon stringRectangle(
            double k,
            const char* s,
            const std::string& justification = "left") const;

        void getWidthsFromBitmap(const Bitmap& bitmap);

        virtual std::string serializeElements(std::string indent = "") const;
        virtual void handleChild(const parse::Node* n);
    };

    /* ! Layer is a type of Node meant to contain a group of similarly drawn objects.  Layer's
         color and matrix property contribute to the matrix and color of each of its decendants. */
    class Layer : public Node
    {
    public:
        Layer();
        virtual ~Layer() {}

        Mat4Property matrix;

        virtual Mat4 getMatrix() const;
    };

    /*! World is a big container that contains Sounds, Sprites and also represents the root
        of a tree of Nodes which can be traversed and drawn by calling draw().*/
    class World : public Layer
    {
        friend class Actor;
    public:
        World();
        virtual ~World();

        Bank* bank;

        PointerVectorProperty<Sampler*> samplers;
        PointerVectorProperty<Sound*> sounds;

        void initSound(AudioPlayer* player);

        virtual void resize(int width, int height);

        void addSampler(Sampler* sampler);
        virtual void removeSampler(const Sampler* sampler);

        void clear();

        Actor* actorInClick(const Vec2& C);

        // Methods of Listener.
        virtual bool mouseDown(const Vec2& C);

        virtual void initWithPath(const char* path);
        virtual void initWithParseNode(const parse::Node* n);
        virtual void handleChild(const parse::Node* n);

        std::string serializeSamplers(std::string indent = "") const;

        virtual void playSound(const std::string& name, double gain = 1.0) const;
        g2c::Sound* getSound(const std::string& name);

        virtual Node* getNode(const std::string& name);
        virtual Sampler* getSampler(const std::string& name);

        virtual void drawInTree(const Mat4& worldMatrix, const Color& worldColor) const;

    private:
        bool soundInitted;

        std::map<std::string, Sampler*> samplerMap;
        std::map<std::string, Sound*> soundMap;
        std::map<std::string, Node*> nodeMap;

        std::vector<Serializable*> deleteResources;

        void buildMaps();
        void updateSamplerMap();
        void updateSoundMap();
        void updateNodeMap(Node* node = NULL);

        // These are const because they get called
        // by playSound (which really should be const).
        void initSoundQueue() const;
        void destroySoundQueue() const;

        mutable AudioPlayer* player;
        mutable Context* context;
        mutable std::vector<Source*> sources;
    };

    /*! Animation is meant to be overridden to attain a motion of some sort.  To use,
        subclass Animation, override begin() step(double) and end(), then instantiate
        by calling new, and add to an Animator's set.  The Animator will call delete.*/
    class Animation {
    friend class Animator;
    public:
        Animation(double instart);
        Animation(double instart, double induration);
        virtual ~Animation();

        /*! If this flag is set, the Animator will forbid user events (keystrokes and mouse clicks
            etc) from reaching any of the listeners in its listener set while this Animation is running.*/
        bool stopsEvents;

        /*! Set to true for an animation that runs indefinitely.*/
        bool forever;

        /*! The Animator sets this flag during the Animation's interval.*/
        bool running;

        double last;

        /*! start and duration determine the interval of time within which the Animation will run.
            Recommend these are set in the constructor and then left alone.*/
        double start;

        /*! start and duration determine the interval of time within which the Animation will run.
            If forever is true, duration determines the scale of the time argument.*/
        double duration;

        /*! The Animator will call advance() every frame while the animation is running.  */
        virtual void advance(double t);

        /*! The Animator will call begin() exactly once when the Animation's interval begins.
            Override to initialize the motion.*/
        virtual void begin();

        /*! The Animator will call step() every frame during the Animation's interval.  The argument
            t varies from 0 to 1 during the time interval */
        virtual void step(double t);

        /*! Unless forever is true, end() will be called exactly once at the end of the
            Animation's interval.*/
        virtual void end();

        void print() const;
        void display() const;

    private:
        int referenceCounter;
    };

    
    /*! Animator is a collection of animations that are running our about to run.  To use,
        instantiate, use add() to add Animation objects, then call step() every frame
        passing an absolute time in seconds.  Animator calls the animations' begin(),
        step() and end() functions at the appropriate times and also destroys the animations
        when they're finished.*/
    class Animator
    {
    public:
        Animator();
        virtual ~Animator();

        int eventStoppedCounter;

        std::set<Animation*> S;
        std::vector<Listener*> listeners;

	/*! Adds an Animation to the Animator.  Animation is meant to be subclassed to get
            custom behavior on its begin() step() and end() calls.  When an animation is
            added, the Animator calls begin(), step() and end() on it from its step() function.*/
        void add(Animation*);

        /*! Removes an Animation from the Animator, so it no longer calls begin() step() and end().*/
        void remove(Animation*);

        /*! Use retain to modify the reference count to an Animation.  This will make it so
            you can be in charge of deleting the Animation yourself, for instance.*/
        void retain(Animation*);

        /*! Use release to modify the reference count of an Animation object.*/
        bool release(Animation*);

        /*! Call this function every frame of animation to have the animator move all its
            Animations forward.*/
        void step(double t);

        /*! Ends all animations.*/
        void end();

        /*! Removes all animations from the Animator.*/
        void clear();

        int animationsAdded;
        int animationsRemoved;
        int animationsRemovedNotInList;
    };

    void setDebugString(std::string* inDebugString);

} // end namespace


#endif
