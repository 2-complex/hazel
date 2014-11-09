
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


#ifndef _GRAPHICS_
#define _GRAPHICS_

#include <vector>
#include <list>
#include <string>
#include <map>
#include <set>

#include "opengl.h"

#include "bank.h"
#include "util.h"
#include "transforms.h"
#include "element.h"
#include "texture.h"


namespace g2c {

class Model;
class Effect;
class Assumption;
class Geometry;
class Shape;
class Buffer;
class IndexBuffer;
class Field;

/*! Uniform properties of Assumptions can be accessed by using the [] operator.  The class Value is
    made to accomodate assignment of a float, vector, matrix or texture value.*/
class Value
{
public:
    Value();
    Value(const Value& v);
    virtual ~Value();
    
    Value(float t);
    Value(const Vec2& t);
    Value(const Vec3& t);
    Value(const Vec4& t);
    Value(const Mat2& t);
    Value(const Mat3& t);
    Value(const Mat4& t);
    Value(const Texture* t);
    
    Value& operator=(const Value& v);
    
    void mimmic(const Value& v);
    
    enum Type
    {
        NONE,
        FLOAT,
        VEC2,
        VEC3,
        VEC4,
        MAT2,
        MAT3,
        MAT4,
        TEXTURE
    } type;
    
    float getFloat() const;
    Vec2 getVec2() const;
    Vec3 getVec3() const;
    Vec4 getVec4() const;
    Mat2 getMat2() const;
    Mat3 getMat3() const;
    Mat4 getMat4() const;
    const Texture* getTexture() const;
    
    void applyToLocation(GLuint location) const;
    std::string toString() const;
    
    std::string textureName;
private:
    union Data
    {
        GLfloat* ptr;
        const Texture* texture;
    } data;
    
    int size;
};

/*! Represents a shader program.  Assign shader code to the members vertexCode and framgnetCode,
    call compile() to compile into a program.  To make the program current, the effect must be
    made the effect of an Assumption, and that assumption must be added to the list of
    assumptions for a Shape.*/
class Effect : public Element
{
friend class Geometry;
friend class Shape;
public:
    Effect();
    virtual ~Effect();
    
    std::string vertexCode;
    std::string fragmentCode;
    
    void compile();
    
protected:
    virtual std::string serializeElements(std::string indent = "") const;
    virtual void handleChild(const parse::Node* n);

private:
    void assume(const std::map<std::string, Value>* assumption) const;
    void use() const;
    
    void bindAttributeToField(const std::string& name, const Field& field) const;
    void disableEnabledAttributes() const;
    
    bool loadShaders();
    void loadVertexShader(const char* s);
    void loadFragmentShader(const char* s);
    bool compileShader(GLuint *shader, GLenum type, const char* code);
    bool linkProgram();
    
    void addAttributesFromProgram();
    void addUniformsFromProgram();
    
    void addAttribute(const char* name);
    void addUniform(const char* name);

    mutable std::vector<GLint> enabledAttributes;
    
    std::map<std::string, GLint> attributeLocationMap;
    std::map<std::string, GLint> uniformLocationMap;
    
    GLuint program;
    GLuint vertShader;
    GLuint fragShader;
};

/*! When a Shape draws, it appeals to its list of assumptions to determine which Effect to use
    and also what parameters to provide to that Effect.

    Assumption is a map of strings to Values, a Value can be either a float, vector, matrix or
    texture.  Assumptions also have a member pointer to an Effect which is optional.  To use
    an Assumption, assign values using the [] operator.  Then add to the list of assumptions
    for a Shape.


    Assumption Use Case 1:  An Assumption can represent a camera in a scene.  In this case,
    Effects in the scene would have have uniform matrices like this:

    uniform mat4 model;
    uniform mat4 model_IT;
    uniform mat4 viewProjection; 

    Then an assumption representing the camera might be initialized like this:

    Assumption camera;
    camera["model"] = Mat4(...);
    camera["model_IT"] = Mat4(...).inverse().transpose();
    camera["viewProjection"] = lookAt(...) * perspective(...);

    All shapes would need to add that Assumption to their assumptions list:
    
    Shape cube;
    cube.assumptions.push_back(camera);


    Assumption Use Case 2:  An Assumption can represent a material.  Since a material is a
    combination of shader, texture, and other parameters, Assumptions have an effect member
    variable for this purpose.  To make a material as an assumption, assign its effect member
    variable and any uniform parameters it needs.

    Assumption wood;
    wood.effect = phongEffect;
    wood["phongCoefficient"] = 0.4;
    wood["texture"] = woodTexture;

    Shapes that are meant to be textured with that material add it to their assumptions list.

    Shape cube;
    cube.assumptions.push_back(camera);
    cube.assumptions.push_back(wood);
    */
class Assumption
    : public std::map<std::string, Value>
    , public Element
{
public:
    Assumption();
    
    bool active;
    std::string effectName;
    Effect* effect;
    
protected:
    virtual std::string serializeElements(std::string indent = "") const;
    virtual void handleChild(const parse::Node* n);
};

/*! Buffers are for storing float data for vertex attributes.  Initialize a Buffer using
    using one of the constructors or by using operator= with a vector or by using the
    one of the overloaded set() functions.  Load per-vertex data as an array of floats, then
    use a Field object to point out repeating patterns to be loaded as vertex attributes.*/
class Buffer : public Element
{
friend class Effect;
public:
    Buffer();
    Buffer(const std::vector<double>& v);
    Buffer(const std::vector<float>& v);
    Buffer(const double* v, int size);
    Buffer(const float* v, int size);
    virtual ~Buffer();

    Buffer& operator=(const std::vector<double>& v);
    Buffer& operator=(const std::vector<float>& v);
    Buffer& set(const double* v, int size);
    Buffer& set(const float* v, int size);


protected:
    virtual std::string serializeElements(std::string indent = "") const;
    virtual void handleChild(const parse::Node* n);

private:
    void initWithFloatArray(const float* v);
    GLuint size;
    GLuint index;
};


/*! IndexBuffer is a list of integer indices.  Initialize with a vector or array of ints or unsigned
    shorts, then assign as the indices member of a Geometry.  Three consecutive indices represent a
    triangle.*/
class IndexBuffer : public Element
{
friend class Geometry;
public:
    IndexBuffer();
    IndexBuffer(const std::vector<int>& v);
    IndexBuffer(const std::vector<unsigned short>& v);
    IndexBuffer(const int* v, int size);
    IndexBuffer(const unsigned short* v, int size);
    virtual ~IndexBuffer();

    IndexBuffer& operator=(const std::vector<int>& v);
    IndexBuffer& operator=(const std::vector<unsigned short>& v);    
    IndexBuffer& set(const int*, int size);
    IndexBuffer& set(const unsigned short*, int size);

protected:
    virtual std::string serializeElements(std::string indent = "") const;
    virtual void handleChild(const parse::Node* n);

private:
    void initWithShortArray(const unsigned short* v);
    GLuint size;
    GLuint index;
};


/*! A Field represents a way of stepping through Buffer to get values to assign to an attribute.
    Each Field has a pointer to a Buffer and a size, stride and offset.  Each of stride, size
    and offset are measured using numbers as the unit, not bytes, so for instance if the buffer
    contains position and normal data for each vertex of a large quad like so:

    -100, -100, 0,  0, 0, 1,
     100, -100, 0,  0, 0, 1,
     100,  100, 0,  0, 0, 1,
    -100,  100, 0,  0, 0, 1

    Fields for position and normal attributes would be something along the lines of:
    positionField = Field(buffer, 3, 6, 0);
    normalField   = Field(buffer, 3, 6, 3);*/
class Field : public Element
{
friend class Effect;
public:
    Field();
    Field(const Buffer* buffer, GLint size, GLsizei stride=0, int offset=0);
    virtual ~Field();
    
    std::string bufferName;
    const Buffer* buffer;
    
protected:
    virtual std::string serializeElements(std::string indent = "") const;
    virtual void handleChild(const parse::Node* n);
    
private:
    GLint size;
    GLsizei stride;
    int offset;
};


/*! Maps the names of vertex attributes to Fields in a buffer encoding those attributes, also
    contains a pointer to an IndexBuffer which stores triples of indices which connect as
    triangles.  A Shape uses a Geometry to draw together with the effect and uniforms it assumes 
    from its assumption list.*/
class Geometry
    : public std::map<std::string, Field>
    , public Element
{
friend class Shape;
public:
    Geometry();
    virtual ~Geometry();
    
    std::string indicesName;
    const IndexBuffer* indices;
    void draw() const;

protected:
    virtual std::string serializeElements(std::string indent = "") const;
    virtual void handleChild(const parse::Node* n);
    
private:
    void bindAttributes(const Effect* effect);
};


/*! Shape connects a Geometry with a list of Assumptions.  To draw, assign the member geometry
    to a Geometry object, and add Assumptions to the assumptions list.  Then call draw().  draw()
    traverses the list of assumptions and selects an Effect to use, then traverses the assumptions
    again gathering unifrom parameters.  The geometry is then drawn using the shader in that Effect
    with the uniforms set to the values encoded in the assumptions list.*/
class Shape : public Element {
public:
    Shape();
    
    /*! Visible flag.  Drawing with visible set to false does nothing.*/
    bool visible;
    
    std::string geometryName;
    std::vector<std::string> assumptionNames;
    
    /*! A pointer to the Geometry to use when drawing.*/
    Geometry* geometry;
    
    /*! A list of pointers to Assumptions.  Shape::draw() iterates through this list and assumes
        an effect and uniform parameters on that effect before drawing the geometry.*/
    std::list<Assumption*> assumptions;
    
    /*! Draw the shape.  Iterates through the assumptions to obtain an Effect and any uniform
        parameters that Effect needs.*/
    void draw() const;

    const Value& get(const std::string& uniformName) const;

protected:
    virtual std::string serializeElements(std::string indent = "") const;
    virtual void handleChild(const parse::Node* n);
};


/*! A Model is a collection of Elements.  Element is the common base class of many of the
    other classes in the graphics library including Element, Texture, Effect, Assumption,
    Geometry, SHape, Buffer, IndexBuffer, and Field.  If the Elements listed refer to each other
    then the Model can be serialized.*/
class Model : public Serializable
{
friend class Element;
friend class Texture2D;
friend class CubeMap;
friend class Effect;
friend class Assumption;
friend class Geometry;
friend class Shape;
friend class Buffer;
friend class IndexBuffer;
friend class Field;

public:
    Model();
    virtual ~Model();

    Bank* bank;

    /*! Draws all the Shapes in the model.*/
    void draw() const;

    /*! Add an Element to the model.  Passing true for flagForDelete also adds the
        element to an internal list of Elements to be deleted when the Model is
        deleted.*/
    Model& add(Element* element, bool flagForDelete = false);

    /*! Add an Element to the model.  Passing true for flagForDelete also adds the
        element to an internal list of Elements to be deleted when the Model is
        deleted.*/
    Model& add(const Model& element, bool flagForDelete = false);

    void makeMaps();
    void compileEffects();
    void resolveNames();

    std::vector<std::string> include;

    std::map<std::string, Texture*> textures;
    std::map<std::string, Assumption*> assumptions;
    std::map<std::string, Effect*> effects;
    std::map<std::string, Buffer*> buffers;
    std::map<std::string, IndexBuffer*> indexBuffers;
    std::map<std::string, Geometry*> geometries;
    std::map<std::string, Shape*> shapes;

protected:
    std::set<Element*> deleteMe;
    std::vector<Element*> elements;

    virtual std::string serializeElements(std::string indent = "") const;
    virtual void deserialize(const std::string& s);
    virtual void handleChild(const parse::Node* n);
};

} // end namespace


#endif
