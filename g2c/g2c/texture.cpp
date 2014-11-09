
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



#include "util.h"
#include "texture.h"

#include <stdlib.h>

using namespace std;

namespace g2c {


Bitmap::Bitmap()
	: data(NULL)
	, width(0)
	, height(0)
	, bitsPerPixel(32)
{
    
}

Bitmap::Bitmap(const Bitmap& b) :
    data(NULL),
    width(b.width),
    height(b.width),
    bitsPerPixel(b.bitsPerPixel)
{
    mimmic(b);
}

void Bitmap::set(
	uint8_t* inData,
	int inWidth,
    int inHeight,
    int inBitsPerPixel)
{
    width = inWidth;
    height = inHeight;
    bitsPerPixel = inBitsPerPixel;
    
    if( data )
        delete[] data;
    
    int size = width * height * bitsPerPixel / 8;
    
    data = new uint8_t[size];
    
    memcpy(data, inData, size * sizeof(uint8_t));
}

void Bitmap::flipVertically()
{
    int rowSize = width * bitsPerPixel / 8;

    uint8_t* tmp = new uint8_t[rowSize];
    
    for( int i = 0; i < height/2; i++ )
    {
        uint8_t* r0 = data + rowSize * i;
        uint8_t* r1 = data + rowSize * (height - i - 1);
        size_t copySize = rowSize * sizeof(uint8_t);

        memcpy(tmp, r0, copySize);
        memcpy(r0, r1, copySize);
        memcpy(r1, tmp, copySize);
    }

    delete[] tmp;
}

void Bitmap::swizzleRGB()
{
    int pd = bitsPerPixel / 8;

    for( int i = 0; i < height * width; i++ )
    {
        uint8_t r = data[pd*i + 0];
        uint8_t g = data[pd*i + 1];
        uint8_t b = data[pd*i + 2];
		
        data[pd*i + 0] = b;
        data[pd*i + 1] = g;
        data[pd*i + 2] = r;
    }
}

void Bitmap::sample(
	double x,
	double y,
	double* color) const
{
	int i0 = x;
	int j0 = y;
	int i1 = x+1;
	int j1 = y+1;
	
	// weights
	double w00 = (i1 - x) * (j1 - y);
	double w10 = (x - i0) * (j1 - y);
	double w01 = (i1 - x) * (y - j0);
	double w11 = (x - i0) * (y - j0);
	
	int p = bitsPerPixel / 8;
	
	for( int channel = 0; channel < bitsPerPixel/8; channel++ )
	{
		if( i1 < 0 )
			i1 = 0;
		if( i1 >= width )
			i1 = width-1;
		if( j1 < 0 )
			j1 = 0;
		if( j1 >= height )
			j1 = height-1;
		
		if( i0 < 0 )
			i0 = 0;
		if( i0 >= width )
			i0 = width-1;
		if( j0 < 0 )
			j0 = 0;
		if( j0 >= height )
			j0 = height-1;
		
		int index_00 = p * (width * j0 + i0) + channel;
		int index_10 = p * (width * j0 + i1) + channel;
		int index_01 = p * (width * j1 + i0) + channel;
		int index_11 = p * (width * j1 + i1) + channel;
		
		color[channel] +=
			w00 * data[index_00]/255.0 +
			w10 * data[index_10]/255.0 +
			w01 * data[index_01]/255.0 +
			w11 * data[index_11]/255.0 ;
	}
}

void Bitmap::resize(int inWidth, int inHeight)
{
	uint8_t* newData = (uint8_t*)calloc(inWidth * inHeight * bitsPerPixel / 8, 1);
	
	// Uninitialized bitmap becomes a black, transparent, 32-bit image.
	if( !data )
	{
		data = newData;
		width = inWidth;
		height = inHeight;
		bitsPerPixel = 32;
		return;
	}
	
	int w = width / inWidth;
	int h = height / inHeight;
	
	double sample_w = 1.0 * width / inWidth;
	double sample_h = 1.0 * height / inHeight;
	
	int p = bitsPerPixel / 8;
	
	// Otherwise, rescale image by sampling each pixel.
	for( int j = 0; j < inHeight; j++ )
	for( int i = 0; i < inWidth; i++ )
    {
    	int index = p * (inWidth * j + i);
    	
    	double color[4] = {0};
    	
    	for( int di = 0; di <= w; di++ )
    	for( int dj = 0; dj <= h; dj++ )
			sample(sample_w * i + di, sample_h * j + dj, color);
    	
    	for( int c = 0; c < p; c++ )
	    	newData[index + c] = 255 * color[c] / ((w+1) * (h+1));
    }
	
	free(data);
	
	width = inWidth;
	height = inHeight;
	data = newData;
}

void Bitmap::mimmic(const Bitmap& b)
{
    set(b.data, b.width, b.height, b.bitsPerPixel);
}

const uint8_t* const Bitmap::getData() const
{
    return data;
}

int Bitmap::getWidth() const
{
    return width;
}

int Bitmap::getHeight() const
{
    return height;
}

int Bitmap::getBitsPerPixel() const
{
    return bitsPerPixel;
}

Bitmap& Bitmap::operator=(const Bitmap& b)
{
    mimmic(b);
    return *this;
}

Bitmap::~Bitmap()
{
    if( data )
    {
        delete[] data;
        data = NULL;
    }
}


Texture::Texture(GLenum target) :
    target(target),
    unit(0),
    index(0),
    bitsPerPixel(0),
    mipmaps(false)
{
}

Texture::Texture(GLenum target, int unit) :
    target(target),
    unit(unit),
    index(0),
    bitsPerPixel(0),
    mipmaps(false)
{
}

Texture::~Texture()
{
    if( index )
    {
        glDeleteTextures(1, &index);
        index = 0;
    }
}

GLuint Texture::getIndex() const
{
    return index;
}

GLuint Texture::getUnit() const
{
    return unit;
}

GLuint Texture::getTarget() const
{
    return target;
}

string Texture::serializeElements(string indent) const
{
    string r = Serializable::serializeElements(indent);
    
    r += TAB + indent + "'unit' : " + intToString(unit) + ",\n";
    r += TAB + indent + "'mipmaps' : " + intToString(mipmaps) + ",\n";
    
    return r;
}

void Texture::handleChild(const parse::Node* n)
{
    Serializable::handleChild(n);
    string n_name = n->data.s;
    parse::Node* value = n->data.value;
    
    if( n_name == "unit" )
        unit = value->data.i;
    
    if( n_name == "mipmaps" )
        mipmaps = value->data.i ? true : false;
}


Texture2D::Texture2D() : Texture(GL_TEXTURE_2D),
    width(0),
    height(0),
    data(NULL)
{
    type = "Texture2D";
}

Texture2D::Texture2D(int unit) : Texture(GL_TEXTURE_2D, unit),
    width(0),
    height(0),
    data(NULL)
{
    type = "Texture2D";
}

Texture2D::~Texture2D()
{
    if( data )
        free(data);
}

void Texture2D::initWithImageData(const GLubyte* inData,
                                  int inWidth,
                                  int inHeight,
                                  int inBitsPerPixel)
{
    if( !index )
        glGenTextures(1, &index);
    
    // glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, index);
    
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    
    width = inWidth;
    height = inHeight;
    bitsPerPixel = inBitsPerPixel;
    
    if( width == 0 || height == 0 )
    {
        g2cerror( "Attempt to initialize texture with zero height or width.\n" );
        g2cerror( "texture: %s\n", name.c_str() );
        exit(0);
    }
    
    if( data )
        free(data);
    
    int size = width * height * bitsPerPixel / 8;
    data = (uint8_t*)malloc(size * sizeof(float));
    
    if( inData )
        memcpy(data, inData, size);
    else
        memset(data, 0, size);
    
    GLenum format = (bitsPerPixel == 24) ? GL_RGB : GL_RGBA;
    glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
    
    if(mipmaps)
    {
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR );
        glGenerateMipmap(GL_TEXTURE_2D);
    }
}

void Texture2D::initWithBitmap(const Bitmap& bitmap)
{
    initWithImageData(
        bitmap.getData(),
        bitmap.getWidth(),
        bitmap.getHeight(),
        bitmap.getBitsPerPixel());
}

string Texture2D::serializeElements(string indent) const
{
    string r = Texture::serializeElements(indent);
    
    r += TAB + indent + "'file' : " + string("'") + file + "',\n";
    
    return r;
}

void Texture2D::handleChild(const parse::Node* n)
{
    Texture::handleChild(n);
    string n_name = n->data.s;
    parse::Node* value = n->data.value;
    
    if( n_name == "file" )
        file = value->data.s;
}



CubeMap::CubeMap() : Texture(GL_TEXTURE_CUBE_MAP)
{
    type = "CubeMap";
}

CubeMap::CubeMap(int unit) : Texture(GL_TEXTURE_CUBE_MAP, unit)
{
    type = "CubeMap";
}

CubeMap::~CubeMap()
{
}

void CubeMap::initWithImageData(const GLubyte* inDataPositiveX,
                                const GLubyte* inDataNegativeX,
                                const GLubyte* inDataPositiveY,
                                const GLubyte* inDataNegativeY,
                                const GLubyte* inDataPositiveZ,
                                const GLubyte* inDataNegativeZ,
                                int inWidth,
                                int inBitsPerPixel)
{
    if( !index )
        glGenTextures(1, &index);
    
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_CUBE_MAP, index);
    
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    const GLubyte* datas[6] = {
        inDataPositiveX,
        inDataNegativeX,
        inDataPositiveY,
        inDataNegativeY,
        inDataPositiveZ,
        inDataNegativeZ
    };
    
    width = inWidth;
    bitsPerPixel = inBitsPerPixel;
    
    if( width == 0 )
    {
        g2cerror( "Attempt to initialize cubemap with zero height or width.\n" );
        g2cerror( "cubemap: %s\n", name.c_str() );
        exit(0);
    }
    
    GLenum format = (bitsPerPixel == 24) ? GL_RGB : GL_RGBA;
    
    for( int i = 0; i < 6; i++ )
    {
        const GLubyte* data = datas[i];
        
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
            0, format, width, width, 0, format, GL_UNSIGNED_BYTE, data);
    }
    
    if(mipmaps)
    {
        glTexParameteri( GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
        glTexParameteri( GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR );
        glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
    }
}

void CubeMap::initWithBitmaps(
	const Bitmap& bitmapPositiveX,
	const Bitmap& bitmapNegativeX,
	const Bitmap& bitmapPositiveY,
	const Bitmap& bitmapNegativeY,
	const Bitmap& bitmapPositiveZ,
	const Bitmap& bitmapNegativeZ)
{
    int w = bitmapPositiveX.getWidth();
    if( w != bitmapPositiveX.getWidth() || w != bitmapPositiveX.getHeight() ||
        w != bitmapNegativeX.getWidth() || w != bitmapNegativeX.getHeight() ||
        w != bitmapPositiveY.getWidth() || w != bitmapPositiveY.getHeight() ||
        w != bitmapNegativeY.getWidth() || w != bitmapNegativeY.getHeight() ||
        w != bitmapPositiveZ.getWidth() || w != bitmapPositiveZ.getHeight() ||
        w != bitmapNegativeZ.getWidth() || w != bitmapNegativeZ.getHeight())
    {
        g2cerror( "Attempt to make cube map from images which are not all the same square size.\n" );
        exit(0);
    }
    
    int bpp = bitmapPositiveX.getBitsPerPixel();
    if( bpp != bitmapPositiveX.getBitsPerPixel() ||
        bpp != bitmapNegativeX.getBitsPerPixel() ||
        bpp != bitmapPositiveY.getBitsPerPixel() ||
        bpp != bitmapNegativeY.getBitsPerPixel() ||
        bpp != bitmapPositiveZ.getBitsPerPixel() ||
        bpp != bitmapNegativeZ.getBitsPerPixel())
    {
        g2cerror( "Attempt to make cube map from images which are not all the same bit-depth.\n" );
        exit(0);
    }
    
    initWithImageData(
        bitmapPositiveX.getData(),
        bitmapNegativeX.getData(),
        bitmapPositiveY.getData(),
        bitmapNegativeY.getData(),
        bitmapPositiveZ.getData(),
        bitmapNegativeZ.getData(), w, bpp);
}

string CubeMap::serializeElements(string indent) const
{
    string r = Texture::serializeElements(indent);
    
    r += TAB + indent + "'positiveXFile' : " + string("'") + positiveXFile + "',\n";
    r += TAB + indent + "'negativeXFile' : " + string("'") + negativeXFile + "',\n";
    r += TAB + indent + "'positiveYFile' : " + string("'") + positiveYFile + "',\n";
    r += TAB + indent + "'negativeYFile' : " + string("'") + negativeYFile + "',\n";
    r += TAB + indent + "'positiveZFile' : " + string("'") + positiveZFile + "',\n";
    r += TAB + indent + "'negativeZFile' : " + string("'") + negativeZFile + "',\n";
    
    return r;
}

void CubeMap::handleChild(const parse::Node* n)
{
    Texture::handleChild(n);
    string n_name = n->data.s;
    parse::Node* value = n->data.value;
    
    if( n_name == "positiveXFile" )
        positiveXFile = value->data.s;
    
    if( n_name == "negativeXFile" )
        negativeXFile = value->data.s;
    
    if( n_name == "positiveYFile" )
        positiveYFile = value->data.s;
    
    if( n_name == "negativeYFile" )
        negativeYFile = value->data.s;
    
    if( n_name == "positiveZFile" )
        positiveZFile = value->data.s;
    
    if( n_name == "negativeZFile" )
        negativeZFile = value->data.s;
}

} // end namespace

