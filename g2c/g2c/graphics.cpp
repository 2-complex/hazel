
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


#include "graphics.h"

#include <stdlib.h>

using namespace std;

namespace g2c {


Value::Value() : type(NONE)
{
    data.ptr = NULL;
}

Value::Value(float t) : type(FLOAT), size(1)
{
    data.ptr = new GLfloat[size];
    data.ptr[0] = t;
}

Value::Value(const Vec2& t) : type(VEC2), size(2)
{
    data.ptr = new GLfloat[size];
    data.ptr[0] = t.x;
    data.ptr[1] = t.y;
}

Value::Value(const Vec3& t) : type(VEC3), size(3)
{
    data.ptr = new GLfloat[size];
    data.ptr[0] = t.x;
    data.ptr[1] = t.y;
    data.ptr[2] = t.z;
}

Value::Value(const Vec4& t) : type(VEC4), size(4)
{
    data.ptr = new GLfloat[size];
    data.ptr[0] = t.x;
    data.ptr[1] = t.y;
    data.ptr[2] = t.z;
    data.ptr[3] = t.w;
}

Value::Value(const Mat2& t) : type(MAT2), size(4)
{
    data.ptr = new GLfloat[size];
    for(int i = 0; i < size; i++)
        data.ptr[i] = (GLfloat)(t.ptr()[i]);
}

Value::Value(const Mat3& t): type(MAT3), size(9)
{
    data.ptr = new GLfloat[size];
    for(int i = 0; i < size; i++)
        data.ptr[i] = (GLfloat)(t.ptr()[i]);
}

Value::Value(const Mat4& t) : type(MAT4), size(16)
{
    data.ptr = new GLfloat[size];
    for(int i = 0; i < size; i++)
        data.ptr[i] = (GLfloat)(t.ptr()[i]);
}

Value::Value(const Texture* t) : type(TEXTURE), size(0)
{
    data.texture = t;
}

Value::~Value()
{
    if(!(type==NONE || type==TEXTURE))
        delete[] data.ptr;
}

Value::Value(const Value& v) : type(NONE), size(0)
{
    mimmic(v);
}

Value& Value::operator=(const Value& v)
{
    mimmic(v);
    return *this;
}

void Value::mimmic(const Value& v)
{
    if(type == NONE)
    {
        type = v.type;
        size = v.size;

        if(type == NONE)
        {
            textureName = v.textureName;
            return;
        }
        else
            textureName = "";

        if(type != TEXTURE)
            data.ptr = new GLfloat[size];
    }

    if(type != v.type)
    {
        g2cerror( "Attempt to set value to incongruous type.\n" );
        exit(0);
    }

    if(type == TEXTURE)
    {
        data.texture = v.data.texture;
    }
    else
    {
        memcpy(data.ptr, v.data.ptr, size*sizeof(GLfloat));
    }
}

void Value::applyToLocation(GLuint location) const
{
    switch(type)
    {
        case NONE:
            g2cerror( "Attempt to bind emtpy value to location." );
            exit(0);
        break;

        case FLOAT:
            glUniform1fv(location, 1, data.ptr);
        break;

        case VEC2:
            glUniform2fv(location, 1, data.ptr);
        break;

        case VEC3:
            glUniform3fv(location, 1, data.ptr);
        break;

        case VEC4:
            glUniform4fv(location, 1, data.ptr);
        break;

        case MAT2:
            glUniformMatrix2fv(location, 1, false, data.ptr);
        break;

        case MAT3:
            glUniformMatrix3fv(location, 1, false, data.ptr);
        break;

        case MAT4:
            glUniformMatrix4fv(location, 1, false, data.ptr);
        break;

        case TEXTURE:
            glActiveTexture(GL_TEXTURE0 + data.texture->getUnit());
            glBindTexture(data.texture->getTarget(), data.texture->getIndex());
            glUniform1i(location, data.texture->getUnit());
        break;
    }
}

float Value::getFloat() const
{
    if( type!=FLOAT )
    {
        g2cerror( "Attempt to get float from value of incongruous type.\n" );
        exit(0);
    }
    GLfloat* t = data.ptr;
    return *t;
}

Vec2 Value::getVec2() const
{
    if( type!=VEC2 )
    {
        g2cerror( "Attempt to get Vec2 from value of incongruous type.\n" );
        exit(0);
    }
    GLfloat* t = data.ptr;
    return Vec2(t[0], t[1]);
}

Vec3 Value::getVec3() const
{
    if( type!=VEC3 )
    {
        g2cerror( "Attempt to get Vec3 from value of incongruous type.\n" );
        exit(0);
    }
    GLfloat* t = data.ptr;
    return Vec3(t[0], t[1], t[2]);
}

Vec4 Value::getVec4() const
{
    if( type!=VEC4 )
    {
        g2cerror( "Attempt to get Vec4 from value of incongruous type.\n" );
        exit(0);
    }
    GLfloat* t = data.ptr;
    return Vec4(t[0], t[1], t[2], t[3]);
}

Mat2 Value::getMat2() const
{
    if( type!=MAT2 )
    {
        g2cerror( "Attempt to get Mat2 from value of incongruous type.\n" );
        exit(0);
    }
    GLfloat* t = data.ptr;
    return Mat2(t[0], t[1],
                t[2], t[3]);
}

Mat3 Value::getMat3() const
{
    if( type!=MAT3 )
    {
        g2cerror( "Attempt to get Mat3 from value of mismatched type.\n" );
        exit(0);
    }
    GLfloat* t = data.ptr;
    return Mat3(t[0], t[1], t[2],
                t[3], t[4], t[5],
                t[6], t[7], t[8]);
}

Mat4 Value::getMat4() const
{
    if( type!=MAT4 )
    {
        g2cerror( "Attempt to get Mat4 from value of mismatched type.\n" );
        exit(0);
    }
    GLfloat* t = data.ptr;
    return Mat4(t[0], t[1], t[2], t[3],
                t[4], t[5], t[6], t[7],
                t[8], t[9], t[10], t[11],
                t[12], t[13], t[14], t[15]);
}

const Texture* Value::getTexture() const
{
    if( type!=TEXTURE )
    {
        g2cerror( "Attempt to get Texture from value of mismatched type.\n" );
        exit(0);
    }
    return data.texture;
}

string Value::toString() const
{
    switch(type)
    {
        case NONE:
            return "None";
        break;

        case FLOAT:
            return floatToString(*(data.ptr));
        break;

        case TEXTURE:
            return "'" + data.texture->name + "'";
        break;

        default:
        {
            string r = "[";
            for(int i = 0; i < size; i++)
                r += floatToString(data.ptr[i]) + ", ";
            r+="]";
            return r;
        }
        break;
    }
}

Effect::Effect() : vertShader(0), fragShader(0), program(0)
{
    type = "Effect";
}

Effect::~Effect()
{
    if( program )
    {
        glDeleteProgram(program);
        program = 0;
    }
    if( vertShader )
    {
        glDeleteShader(vertShader);
        vertShader = 0;
    }
    if( fragShader )
    {
        glDeleteShader(fragShader);
        fragShader = 0;
    }
}

void Effect::compile()
{
    loadShaders();
    addUniformsFromProgram();
    addAttributesFromProgram();
}

void Effect::addAttributesFromProgram()
{
    int maxNameLength, numberAttributes;
    glGetProgramiv(program, GL_ACTIVE_ATTRIBUTE_MAX_LENGTH, &maxNameLength);
    glGetProgramiv(program, GL_ACTIVE_ATTRIBUTES, &numberAttributes);
    char* name = new char[maxNameLength];

    GLsizei length = 0;
    GLint size = 0;
    GLenum type = 0;

    for( int i = 0; i < numberAttributes; i++ )
    {
        glGetActiveAttrib(program, i, maxNameLength, &length, &size, &type, name);
        addAttribute(name);
    }

    delete[] name;
}

void Effect::addUniformsFromProgram()
{
    int maxNameLength, numberUniforms;
    glGetProgramiv(program, GL_ACTIVE_UNIFORM_MAX_LENGTH, &maxNameLength);
    glGetProgramiv(program, GL_ACTIVE_UNIFORMS, &numberUniforms);

    char* name = new char[maxNameLength+1];
    memset(name, 0, maxNameLength+1);

    GLsizei length = 0;
    GLint size = 0;
    GLenum type = 0;

    for( int i = 0; i < numberUniforms; i++ )
    {
        glGetActiveUniform(program, i, maxNameLength, &length, &size, &type, name);
        addUniform(name);
    }

    delete[] name;
}

void Effect::assume(const map<string, Value>* assumption) const
{
    glUseProgram(program);
    for(map<string, Value>::const_iterator itr = assumption->begin();
        itr!=assumption->end();
        itr++)
    {
        map<string, GLint>::const_iterator mitr = uniformLocationMap.find(itr->first);
        if( mitr!=uniformLocationMap.end() )
            itr->second.applyToLocation(mitr->second);
        }
    }

bool Effect::loadShaders()
{
    program = glCreateProgram();

    const string header =
#if defined(ANDROID)
        "precision mediump float;\n";
#else
        "#ifdef GL_ES\nprecision highp float;\n#endif\n";
#endif

    loadVertexShader(vertexCode.c_str());
    loadFragmentShader((header+fragmentCode).c_str());

    if (!linkProgram())
        return false;

    return true;
}


void Effect::bindAttributeToField(const string& name, const Field& field) const
{
    if( !field.buffer )
    {
        g2cerror( "Attempt to bind attributes to field with no buffer\n" );
        exit(0);
    }
    glBindBuffer(GL_ARRAY_BUFFER, field.buffer->index);

    map<string, GLint>::const_iterator itr = attributeLocationMap.find(name);
    if(itr == attributeLocationMap.end())
    {
        g2cerror( "Attempt to bind to non-existant attribute: %s\n", name.c_str() );
        exit(0);
    }
    GLuint attributeLocaion = itr->second;

    glEnableVertexAttribArray(attributeLocaion);
    enabledAttributes.push_back(attributeLocaion);

    if( field.size < 1 || field.size > 4 )
    {
        g2cerror( "Attempt to draw with field size not 1 2 3 or 4.\n" );
        exit(0);
    }

    glVertexAttribPointer(attributeLocaion, field.size, GL_FLOAT, false,
        field.stride*sizeof(float), (const GLvoid*)(field.offset*sizeof(float)));
}

void Effect::disableEnabledAttributes() const
{
    for(vector<GLint>::iterator itr = enabledAttributes.begin();
        itr!=enabledAttributes.end();
        itr++)
    {
        glDisableVertexAttribArray(*itr);
    }
    enabledAttributes.clear();
}


bool Effect::compileShader(GLuint *shader, GLenum type, const char* code)
{
    GLint status;

    *shader = glCreateShader(type);
    glShaderSource(*shader, 1, &code, NULL);
    glCompileShader(*shader);

    GLint logLength;
    glGetShaderiv(*shader, GL_INFO_LOG_LENGTH, &logLength);
    if( logLength > 0 )
    {
        GLchar *log = (GLchar *)malloc(logLength);
        glGetShaderInfoLog(*shader, logLength, &logLength, log);
        if( logLength )
            g2clog("Shader compile log:\n%s", log);
        free(log);
    }

    glGetShaderiv(*shader, GL_COMPILE_STATUS, &status);
    if( !status )
    {
        glDeleteShader(*shader);
        return false;
    }

    return true;
}

bool Effect::linkProgram()
{
    GLint status;

    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &status);

    if (status == 0)
        return false;

    return true;
}

void Effect::addAttribute(const char* name)
{
    attributeLocationMap[name] = glGetAttribLocation(program, name);
}

void Effect::addUniform(const char* name)
{
    GLint location = glGetUniformLocation(program, name);

    if(location<0)
    {
        g2cerror( "location turned up negative for uniform: %s\n", name );
        exit(0);
    }
    uniformLocationMap[name] = location;
}

void Effect::loadVertexShader(const char* s)
{
    compileShader(&vertShader, GL_VERTEX_SHADER, s);
    glAttachShader(program, vertShader);
}

void Effect::loadFragmentShader(const char* s)
{
    compileShader(&fragShader, GL_FRAGMENT_SHADER, s);
    glAttachShader(program, fragShader);
}

void Effect::use() const
{
    glUseProgram(program);
}

string Effect::serializeElements(string indent) const
{
    string r = Serializable::serializeElements(indent);

    r += TAB + indent + "'vertexCode' : '" + vertexCode + "',\n";
    r += TAB + indent + "'fragmentCode' : '" + fragmentCode + "',\n";

    return r;
}

void Effect::handleChild(const parse::Node* n)
{
    Serializable::handleChild(n);

    string n_name = n->data.s;
    parse::Node* value = n->data.value;

    if( n_name == "vertexCode" )
    {
        vertexCode = value->data.s;
    }

    if( n_name == "fragmentCode" )
    {
        fragmentCode = value->data.s;
    }

    if( !(fragmentCode=="" || vertexCode=="") )
    {
        loadShaders();
    }
}

Assumption::Assumption() : effect(NULL), active(true)
{
    type = "Assumption";
}

string Assumption::serializeElements(string indent) const
{
    string r = Serializable::serializeElements(indent);

    if( effect )
        r += TAB + indent + "'effect' : '" + effect->name + "',\n";

    for(const_iterator itr = begin(); itr!=end(); itr++)
        r += TAB + indent + "'" + itr->first + "' : " + itr->second.toString() + ",\n";

    return r;
}

void Assumption::handleChild(const parse::Node* n)
{
    Serializable::handleChild(n);

    string n_name = n->data.s;
    parse::Node* value = n->data.value;

    if(n_name == "name" || n_name == "type")
        return;

    if(n_name == "effect")
    {
        effectName = value->data.s;
    }
    else if(n_name == "active")
    {
        active = value->data.i;
    }
    else if(value->data.s != "")
    {
        // If it's a string, we assume that string is the name of a texture.
        // We store the string for now and later will resolve to a texture object.
        Value v;
        v.textureName = value->data.s;
        (*this)[n_name] = v;
    }
    else if(!value->children.empty())
    {
        // If it's not a string placeholder for a texture, we check if it has children in
        // which case, we assume it's a vector or a matrix.
        vector<double> v;
        for(vector<parse::Node*>::const_iterator itr = value->children.begin();
            itr!=value->children.end();
            itr++)
        {
            v.push_back((*itr)->data.x);
        }
        switch(v.size())
        {
            case 2:
                (*this)[n_name] = Vec2(v[0], v[1]);
            break;

            case 3:
                (*this)[n_name] = Vec3(v[0], v[1], v[2]);
            break;

            // This is a design flaw.  A four entry array here could also mean a 2x2 matrix.
            case 4:
                (*this)[n_name] = Vec4(v[0], v[1], v[2], v[3]);
            break;

            case 9:
                (*this)[n_name] = Mat3(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]);
            break;

            case 16:
                (*this)[n_name] = Mat4(
                    v[0], v[1], v[2], v[3],
                                       v[4], v[5], v[6], v[7],
                                       v[8], v[9], v[10], v[11],
                                       v[12], v[13], v[14], v[15]);
            break;

            default:
                g2clog( "Value of %s of unusual size : %d\n", n_name.c_str(), (int)(v.size()) );
                exit(0);
            break;
        }
    }
    else
    {
        // Otherwise we assume it's a number.
        (*this)[n_name] = value->data.x;
    }
}

Buffer::Buffer() : index(0), size(0)
{
    type = "Buffer";
}

Buffer::Buffer(const vector<double>& v)
{
    type = "Buffer";
    size = v.size();
    float* fv = new float[size];
    for(int i = 0; i < size; i++)
        fv[i] = v[i];

    size = v.size();
    initWithFloatArray(fv);

    delete[] fv;
}

Buffer::Buffer(const vector<float>& v)
{
    type = "Buffer";
    size = v.size();
    float* fv = new float[size];
    for(int i = 0; i < size; i++)
        fv[i] = v[i];

    initWithFloatArray(fv);
    delete[] fv;
}

Buffer::Buffer(const double* v, int size) : size(size)
{
    type = "Buffer";
    float* fv = new float[size];
    for(int i = 0; i < size; i++)
        fv[i] = v[i];

    initWithFloatArray(fv);
    delete[] fv;
}

Buffer::Buffer(const float* v, int size) : size(size)
{
    type = "Buffer";
    initWithFloatArray(v);
}



Buffer::~Buffer()
{
    if( index )
        glDeleteBuffers(1, &index);
}

Buffer& Buffer::operator=(const vector<double>& v)
{
    type = "Buffer";
    size = v.size();
    float* fv = new float[size];
    for(int i = 0; i < size; i++)
        fv[i] = v[i];
    
    size = v.size();
    initWithFloatArray(fv);
    
    delete[] fv;
    return *this;
}

Buffer& Buffer::operator=(const vector<float>& v)
{
    type = "Buffer";
    size = v.size();
    float* fv = new float[size];
    for(int i = 0; i < size; i++)
        fv[i] = v[i];
    
    initWithFloatArray(fv);
    delete[] fv;

    return *this;
}


Buffer& Buffer::set(const float* v, int size)
{
    this->size = size;
    initWithFloatArray(v);
    return *this;
}


Buffer& Buffer::set(const double* v, int size)
{
    this->size = size;

    type = "Buffer";
    float* fv = new float[size];
    for(int i = 0; i < size; i++)
        fv[i] = v[i];

    initWithFloatArray(fv);
    delete[] fv;
    return *this;
}


string Buffer::serializeElements(string indent) const
{
#if !(defined(TARGET_OS_IPHONE) || defined(ANDROID))
    string r = Serializable::serializeElements(indent);

    float* array = new float[size];
    glGetBufferSubData(GL_ARRAY_BUFFER, 0, size*sizeof(float), array);


    r += TAB + indent + "'array' : [";
    for(int i = 0; i < size; i++)
    {
        if( i%8 == 0 )
            r += "\n" + TAB + TAB + indent;
        r += floatToString(array[i]) + ", ";
    }
    r += "\n" + TAB + indent + "],\n";

    delete[] array;

    return r;
#else
    return "(Buffer::serialize does not work on mobile.)";
#endif
}

void Buffer::handleChild(const parse::Node* n)
{
    Serializable::handleChild(n);
    string n_name = n->data.s;
    parse::Node* value = n->data.value;

    if( n_name == "array" )
    {
        size = value->children.size();
        float* array = new float[size];

        int i = 0;
        for(vector<parse::Node*>::const_iterator itr = value->children.begin();
            itr!=value->children.end();
            itr++)
        {
            array[i] = (*itr)->data.x;
            i++;
        }

        initWithFloatArray(array);

        delete[] array;
    }
}

void Buffer::initWithFloatArray(const float* v)
{
    if( index )
        glDeleteBuffers(1, &index);
    glGenBuffers(1, &index);

    glBindBuffer(GL_ARRAY_BUFFER, index);
    glBufferData(GL_ARRAY_BUFFER, size * sizeof(GLfloat), v, GL_STREAM_DRAW);
}

IndexBuffer::IndexBuffer() : index(0), size(0)
{
    type = "IndexBuffer";
}

IndexBuffer::IndexBuffer(const vector<int>& v)
{
    type = "IndexBuffer";
    size = v.size();
    unsigned short* sv = new unsigned short[size];
    for(int i = 0; i < size; i++)
        sv[i] = v[i];

    size = v.size();
    initWithShortArray(sv);

    delete[] sv;
}

IndexBuffer::IndexBuffer(const vector<unsigned short>& v)
{
    type = "IndexBuffer";
    size = v.size();
    unsigned short* sv = new unsigned short[size];
    for(int i = 0; i < size; i++)
        sv[i] = v[i];

    initWithShortArray(sv);
    delete[] sv;
}

IndexBuffer::IndexBuffer(const int* v, int size) : size(size)
{
    type = "IndexBuffer";
    unsigned short* sv = new unsigned short[size];
    for(int i = 0; i < size; i++)
        sv[i] = v[i];

    initWithShortArray(sv);
    delete[] sv;
}

IndexBuffer::IndexBuffer(const unsigned short* v, int size) : size(size)
{
    type = "IndexBuffer";
    initWithShortArray(v);
}

IndexBuffer::~IndexBuffer()
{
    if( index )
        glDeleteBuffers(1, &index);
}

IndexBuffer& IndexBuffer::operator=(const std::vector<int>& v)
{
    type = "IndexBuffer";
    size = v.size();
    unsigned short* sv = new unsigned short[size];
    for(int i = 0; i < size; i++)
        sv[i] = v[i];
    
    size = v.size();
    initWithShortArray(sv);
    
    delete[] sv;

    return *this;
}

IndexBuffer& IndexBuffer::operator=(const std::vector<unsigned short>& v)    
{
    type = "IndexBuffer";
    size = v.size();
    unsigned short* sv = new unsigned short[size];
    for(int i = 0; i < size; i++)
        sv[i] = v[i];
    
    size = v.size();
    initWithShortArray(sv);
    
    delete[] sv;

    return *this;
}

IndexBuffer& IndexBuffer::set(const int* v, int size)
{
    this->size = size;
    
    unsigned short* sv = new unsigned short[size];
    for(int i = 0; i < size; i++)
        sv[i] = v[i];
    
    initWithShortArray(sv);
    delete[] sv;
    
    return *this;
}

IndexBuffer& IndexBuffer::set(const unsigned short* v, int size)
{
    this->size = size;
    initWithShortArray(v);

    return *this;
}


string IndexBuffer::serializeElements(string indent) const
{
#if !(defined(TARGET_OS_IPHONE) || defined(ANDROID))
    string r = Serializable::serializeElements(indent);

    unsigned short* array = new unsigned short[size];
    glGetBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, size*sizeof(unsigned short), array);

    r += TAB + indent + "'array' : [";
    for(int i = 0; i < size; i++)
    {
        if( i%3 == 0 )
            r += "\n" + TAB + TAB + indent;
        r += intToString(array[i]) + ", ";
    }
    r += "\n" + TAB + indent + "],\n";

    delete[] array;
    return r;
#else
    return "(IndexBuffer::serialize does not work on mobile.)";
#endif
}

void IndexBuffer::handleChild(const parse::Node* n)
{
    Serializable::handleChild(n);
    string n_name = n->data.s;
    parse::Node* value = n->data.value;

    if( n_name == "array" )
    {
        size = value->children.size();
        unsigned short* array = new unsigned short[size];

        int i = 0;
        for(vector<parse::Node*>::const_iterator itr = value->children.begin();
            itr!=value->children.end();
            itr++)
        {
            array[i] = (*itr)->data.i;
            i++;
        }

        initWithShortArray(array);

        delete[] array;
    }
}

void IndexBuffer::initWithShortArray(const unsigned short* v)
{
    if( index )
        glDeleteBuffers(1, &index);
    glGenBuffers(1, &index);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
        size*sizeof(unsigned short), (GLshort*)v, GL_STREAM_DRAW);
}


Geometry::Geometry() : indices(NULL)
{
    type = "Geometry";
}

Geometry::~Geometry()
{

}

string Geometry::serializeElements(string indent) const
{
    string r = Serializable::serializeElements(indent);

    if(indices)
        r += TAB + indent + "'indices' : '" + indices->name + "',\n";

    for(const_iterator itr = begin(); itr!=end(); itr++)
    {
        r += TAB + indent + "'" + itr->first + "' : " +
            flattenWhitespace(itr->second.serialize(indent + TAB).c_str()) + ",\n";
    }

    return r;
}

void Geometry::handleChild(const parse::Node* n)
{
    Serializable::handleChild(n);
    string n_name = n->data.s;
    parse::Node* value = n->data.value;

    if( n_name == "name" || n_name == "type" )
        return;

    if( n_name == "indices" )
    {
        indicesName = value->data.s;
    }
    else
    {
        (*this)[n_name].initWithParseNode(value);
    }
}

void Geometry::bindAttributes(const Effect* effect)
{
    if(!effect)
    {
        g2cerror( "Attributes bound to null effect\n" );
        exit(0);
    }

    for(const_iterator itr = begin(); itr!=end(); itr++)
    {
        effect->bindAttributeToField(itr->first, itr->second);
    }
}

void Geometry::draw() const
{
    if( !indices )
    {
        g2cerror( "Attempt to draw geometry with undefined indices.\n" );
        exit(0);
    }

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices->index);
    glDrawElements(GL_TRIANGLES, indices->size, GL_UNSIGNED_SHORT, 0);
}

Field::Field() : buffer(NULL), size(0), stride(0), offset(0)
{
    type = "Field";
}

Field::Field(const Buffer* buffer, GLint size, GLsizei stride, int offset) :
    buffer(buffer), size(size), stride(stride), offset(offset)
{
    type = "Field";
}

Field::~Field()
{

}

string Field::serializeElements(std::string indent) const
{
    string r = Serializable::serializeElements(indent);

    if( buffer )
        r += TAB + indent + "'buffer' : '" + buffer->name + "',\n";

    r += TAB + indent + "'size' : " + intToString(size) + ",\n";
    r += TAB + indent + "'stride' : " + intToString(stride) + ",\n";
    r += TAB + indent + "'offset' : " + intToString(offset) + ",\n";

    return r;
}

void Field::handleChild(const parse::Node* n)
{
    Serializable::handleChild(n);
    string n_name = n->data.s;
    parse::Node* value = n->data.value;

    if( n_name == "buffer" )
        bufferName = value->data.s;

    if( n_name == "size" )
        size = value->data.i;

    if( n_name == "stride" )
        stride = value->data.i;

    if( n_name == "offset" )
        offset = value->data.i;
}

Shape::Shape() : geometry(NULL), visible(true)
{
    type = "Shape";
}

Value noValue;

const Value& Shape::get(const string& uniformName) const
{
    for(list<Assumption*>::const_reverse_iterator itr = assumptions.rbegin();
        itr!=assumptions.rend();
        itr++)
    {
        const Assumption* assumption(*itr);
        if( assumption->active )
        {
            for(map<string, Value>::const_iterator itr = assumption->begin();
                itr!=assumption->end();
                itr++)
            {
                if( itr->first == uniformName )
                    return itr->second;
            }
        }
    }

    return noValue;
}

void Shape::draw() const
{
    if(!visible)
        return;

    if(!geometry)
    {
        g2cerror( "Shape drawn with no geometry.\n" );
        exit(0);
    }

    const Effect* effect = NULL;

    // Search through assumptions looking for effect.
    for(list<Assumption*>::const_reverse_iterator itr = assumptions.rbegin();
        itr!=assumptions.rend();
        itr++)
    {
        if( (*itr)->active && (*itr)->effect )
        {
            effect = (*itr)->effect;
            break;
        }
    }

    // If we didn't find an effect, bail.
    if(!effect)
    {
        g2cerror( "Shape %s drawn with no effect.\n", name.c_str() );
        exit(0);
    }

    // If we did find an effect, use it.
    effect->use();

    // Iterate through the assumptions and assume the uniforms in them.
    for(list<Assumption*>::const_iterator itr = assumptions.begin();
        itr!=assumptions.end();
        itr++)
    {
        if( (*itr)->active )
            effect->assume(*itr);
    }

    geometry->bindAttributes(effect);
    geometry->draw();
    effect->disableEnabledAttributes();

    glUseProgram(0);
}


string Shape::serializeElements(string indent) const
{
    string r = Serializable::serializeElements(indent);

    if( !visible )
        r += TAB + indent + "'visible' : '" + intToString(visible) + "',\n";

    if( geometry )
        r += TAB + indent + "'geometry' : '" + geometry->name + "',\n";
    r += TAB + indent + "'assumptions' : [\n";

    for(list<Assumption*>::const_iterator itr = assumptions.begin(); itr!=assumptions.end(); itr++)
        r += TAB + TAB + indent + "'" + (*itr)->name + "',\n";

    r += TAB + indent + "],\n";

    return r;
}

void Shape::handleChild(const parse::Node* n)
{
    Serializable::handleChild(n);
    string n_name = n->data.s;
    parse::Node* value = n->data.value;

    if( n_name == "visible" )
        visible = value->data.i;

    if( n_name == "geometry" )
        geometryName = value->data.s;

    if( n_name == "assumptions" )
    {
        for(vector<parse::Node*>::const_iterator itr = value->children.begin();
            itr!=value->children.end();
            itr++)
        {
            assumptionNames.push_back((*itr)->data.s);
        }
    }
}



Model::Model()
{
    type = "Model";
}

Model::~Model()
{
    for(set<Element*>::iterator itr = deleteMe.begin(); itr!=deleteMe.end(); itr++)
        delete *itr;
}

void Model::draw() const
{
    for(map<string, Shape*>::const_iterator itr = shapes.begin(); itr!=shapes.end(); itr++)
        itr->second->draw();
}

Model& Model::add(Element* element, bool flagForDelete)
{
    elements.push_back(element);

    if( element->type == "Effect" )
        effects[element->name] = (Effect*)element;
    else if( element->type == "Assumption" )
        assumptions[element->name] = (Assumption*)element;
    else if( element->type == "Buffer" )
        buffers[element->name] = (Buffer*)element;
    else if( element->type == "IndexBuffer" )
        indexBuffers[element->name] = (IndexBuffer*)element;
    else if( element->type == "Geometry" )
        geometries[element->name] = (Geometry*)element;
    else if( element->type == "Shape" )
        shapes[element->name] = (Shape*)element;
    else if( element->type == "Texture" || element->type == "Texture2D" || element->type == "CubeMap" )
        textures[element->name] = (Texture*)element;
    else
    {
        g2cerror( "Element not inserted" );
        exit(0);
    }

    if(flagForDelete)
        deleteMe.insert(element);

    return *this;
}

Model& Model::add(const Model& model, bool flagForDelete)
{
    for(vector<Element*>::const_iterator itr = model.elements.begin(); itr != model.elements.end(); itr++)
        add(*itr, flagForDelete);
    return *this;
}

void Model::resolveNames()
{
    // Iterate through assumptions (recall assumptions are maps from strings to values)
    // and find Values that should be assigned to textures by name.
    for(map<string, Assumption*>::iterator itr = assumptions.begin();
        itr!=assumptions.end();
        itr++)
    {
        // Also if the assumption has an effect referenced by name, assign that effect.
        Assumption* assumption = itr->second;
        const string& effectName = assumption->effectName;
        if( effectName!="" )
        {
            map<string, Effect*>::iterator mitr = effects.find(effectName);
            if(mitr == effects.end())
            {
                g2cerror( "effect not found %s\n", effectName.c_str() );
                exit(0);
            }
            else
                assumption->effect = mitr->second;
        }

        for(Assumption::iterator itr = assumption->begin(); itr!=assumption->end(); itr++)
        {
            Value& value(itr->second);
            const string& textureName = itr->second.textureName;

            if( textureName!="" )
            {
                map<string, Texture*>::iterator mitr = textures.find(textureName);
                if(mitr == textures.end())
                {
                    g2cerror( "texture not found %s\n", textureName.c_str() );
                    exit(0);
                }
                else
                    value = mitr->second;
            }
        }
    }

    // Iterate through geometries.  Geometries have an index buffer and a map to fields.  Each
    // field has a name of a buffer that needs to be resolved.
    for(map<string, Geometry*>::iterator itr = geometries.begin(); itr!=geometries.end(); itr++)
    {
        Geometry* geometry = itr->second;
        const string& indicesName(geometry->indicesName);
        if(indicesName!="")
        {
            map<string, IndexBuffer*>::iterator mitr = indexBuffers.find(indicesName);
            if(mitr == indexBuffers.end())
            {
                g2cerror( "indexbuffer not found %s\n", indicesName.c_str() );
                exit(0);
            }
            else
                geometry->indices = mitr->second;
        }

        for(Geometry::iterator itr = geometry->begin(); itr!=geometry->end(); itr++)
        {
            string& bufferName = itr->second.bufferName;
            map<string, Buffer*>::iterator mitr = buffers.find(bufferName);
            if(mitr == buffers.end())
            {
                g2cerror( "buffer not found %s\n", bufferName.c_str() );
                exit(0);
            }
            else
                itr->second.buffer = mitr->second;
        }
    }

    // Iterate through the shapes next and resolve their geometries and assumptions.
    for(map<string, Shape*>::iterator itr = shapes.begin(); itr!=shapes.end(); itr++)
    {
        Shape* shape = itr->second;
        const string& geometryName(shape->geometryName);
        if(geometryName!="")
        {
            map<string, Geometry*>::iterator mitr = geometries.find(geometryName);
            if(mitr == geometries.end())
            {
                g2cerror( "geometry not found %s\n", geometryName.c_str() );
                exit(0);
            }
            else
                shape->geometry = mitr->second;
        }

        for(vector<string>::iterator itr = shape->assumptionNames.begin();
            itr != shape->assumptionNames.end();
            itr++)
        {
            string& assumptionName = *itr;
            map<string, Assumption*>::iterator mitr = assumptions.find(assumptionName);
            if(mitr == assumptions.end())
            {
                g2cerror( "assumption not found %s\n", assumptionName.c_str() );
                exit(0);
            }
            else
                shape->assumptions.push_back(mitr->second);
        }
    }
}

string Model::serializeElements(string indent) const
{
    string r = Serializable::serializeElements();

    if( !include.empty() )
    {
        r += TAB + indent + "'include' : [\n";
        for(vector<string>::const_iterator itr = include.begin(); itr!=include.end(); itr++)
            r += TAB + TAB + indent + "'" + (*itr) + "',\n";
        r += TAB + indent +  "],\n";
    }

    if( !elements.empty() )
    {
        r += TAB + indent + "'elements' : [\n";
        for(vector<Element*>::const_iterator itr = elements.begin(); itr!=elements.end(); itr++)
            r += (*itr)->serialize(TAB + TAB + indent) + ",\n";
        r += TAB + indent +  "],\n";
    }

    return r;
}

void Model::deserialize(const std::string& s)
{
    Serializable::deserialize(s);

    for(vector<string>::iterator itr = include.begin(); itr != include.end(); itr++)
    {
        Model model;
        bank->initSerializableWithPath(&model, itr->c_str());
        model.deleteMe.clear();
        add(model, true);
    }

    compileEffects();
    resolveNames();
}

void Model::compileEffects()
{
    for(map<string, Effect*>::const_iterator itr = effects.begin();
        itr!=effects.end();
        itr++)
    {
        itr->second->compile();
    }
}

void Model::handleChild(const parse::Node* n)
{
    Serializable::handleChild(n);
    string n_name = n->data.s;
    parse::Node* value = n->data.value;

    if( n_name == "include" )
    {
        for(vector<parse::Node*>::const_iterator itr = value->children.begin();
            itr!=value->children.end();
            itr++)
        {
            string filename = (*itr)->data.s;
            include.push_back(filename);
        }
    }

    if( n_name == "elements" )
    {
        for(vector<parse::Node*>::const_iterator itr = value->children.begin();
            itr!=value->children.end();
            itr++)
        {
            parse::Node* element = *itr;
            string elementType = getType(element);

            Texture2D* newTexture = NULL;
            CubeMap* newCubeMap = NULL;
            Element* t = NULL;

            if( elementType == "Texture" || elementType == "Texture2D" )
                t = newTexture = new Texture2D;

            if( elementType == "Assumption" )
                t = new Assumption;

            if( elementType == "Buffer" )
                t = new Buffer;

            if( elementType == "CubeMap" )
                t = newCubeMap = new CubeMap;

            if( elementType == "IndexBuffer" )
                t = new IndexBuffer;

            if( elementType == "Effect" )
                t = new Effect;

            if( elementType == "Geometry" )
                t = new Geometry;

            if( elementType == "Shape" )
                t = new Shape;

            t->initWithParseNode(element);
            add(t, true);

            if( elementType == "Texture2D" ||
                elementType == "Texture" )
            {
                if( newTexture->file != "" )
                    bank->initTextureWithPath(newTexture, newTexture->file.c_str());
            }

            if( elementType == "CubeMap" )
            {
                if( newCubeMap->positiveXFile != "" &&
                    newCubeMap->negativeXFile != "" &&
                    newCubeMap->positiveYFile != "" &&
                    newCubeMap->negativeYFile != "" &&
                    newCubeMap->positiveZFile != "" &&
                    newCubeMap->negativeZFile != "" )
                {
                    Bitmap positiveXBitmap,
                           negativeXBitmap,
                           positiveYBitmap,
                           negativeYBitmap,
                           positiveZBitmap,
                           negativeZBitmap;

                    bank->initBitmapWithPath(&positiveXBitmap, newCubeMap->positiveXFile.c_str());
                    bank->initBitmapWithPath(&negativeXBitmap, newCubeMap->negativeXFile.c_str());
                    bank->initBitmapWithPath(&positiveYBitmap, newCubeMap->positiveYFile.c_str());
                    bank->initBitmapWithPath(&negativeYBitmap, newCubeMap->negativeYFile.c_str());
                    bank->initBitmapWithPath(&positiveZBitmap, newCubeMap->positiveZFile.c_str());
                    bank->initBitmapWithPath(&negativeZBitmap, newCubeMap->negativeZFile.c_str());
                    newCubeMap->initWithBitmaps(
                        positiveXBitmap,
                        negativeXBitmap,
                        positiveYBitmap,
                        negativeYBitmap,
                        positiveZBitmap,
                        negativeZBitmap);
                }
            }
        }
    }
}

} // end namespace

