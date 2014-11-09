
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


#include "serializable.h"
#include "util.h"

#include "log.h"

#include <vector>

using namespace std;

namespace g2c {


Serializable::Property::Property(PropertyType type, const std::string& name, long offset) :
    type(type),
    name(name),
    offset(offset) {}

void Serializable::deserialize(const std::string& s)
{
    parse::Node* n = new parse::Node(s.c_str());
    initWithParseNode(n);
    delete n;
}

void Serializable::initWithParseNode(const parse::Node* n)
{
    for(vector<parse::Node*>::const_iterator itr = n->children.begin();
        itr!=n->children.end();
        itr++)
    {
        const parse::Node* node = *itr;
        if(node->type == parse::kString && node->data.value!=NULL)
            handleChild(node);
    }
}

void Serializable::handleChild(const parse::Node* n)
{
    string n_name = n->data.s;
    parse::Node* value = n->data.value;
    
    if( n_name == "name" )
        name = value->data.s;
    
    if( n_name == "type" )
        type = value->data.s;
    
    for(vector<Property>::iterator itr = properties.begin();
        itr!=properties.end();
        itr++ )
    {
        void* ptr = (char*)(this) + itr->offset;
        
        switch( itr->type )
        {
            case kInt:
                if( n_name == itr->name )
                    *(reinterpret_cast<int*>(ptr)) = value->data.i;
            break;
            
            case kDouble:
                if( n_name == itr->name )
                    *(reinterpret_cast<double*>(ptr)) = value->data.x;
            break;
            
            case kString:
                if( n_name == itr->name )
                    *(reinterpret_cast<string*>(ptr)) = value->data.s;
            break;
            
            case kObject:
                if( n_name == itr->name )
                    reinterpret_cast<Serializable*>(ptr)->initWithParseNode(value);
            break;
            
            case kVectorDouble:
                if( n_name == itr->name )
                {
                    vector<double>* vptr = (vector<double>*)ptr;
                    for(vector<parse::Node*>::const_iterator itr = value->children.begin();
                        itr != value->children.end();
                        itr++)
                    {
                        vptr->push_back( (*itr)->data.x );
                    }
                }
            break;
            
            case kVectorInt:
                if( n_name == itr->name )
                {
                    vector<int>* vptr = (vector<int>*)ptr;
                    for(vector<parse::Node*>::const_iterator itr = value->children.begin();
                        itr != value->children.end();
                        itr++)
                    {
                        vptr->push_back( (*itr)->data.i );
                    }
                }
            break;
            
            case kVectorString:
                if( n_name == itr->name )
                {
                    vector<string>* vptr = (vector<string>*)ptr;
                    for(vector<parse::Node*>::const_iterator itr = value->children.begin();
                        itr != value->children.end();
                        itr++)
                    {
                        vptr->push_back( (*itr)->data.s );
                    }
                }
            break;
        }
    }
}

std::string Serializable::getType(const parse::Node* n) const
{
    string type = "";
    for(vector<parse::Node*>::const_iterator itr = n->children.begin();
        itr!=n->children.end();
        itr++)
    {
        if((*itr)->data.s == "type")
        {
            type = (*itr)->data.value->data.s;
            break;
        }
    }
    return type;
}

string Serializable::serialize(string indent) const
{
    return serializeBegin(indent) +
        serializeElements(indent) +
        serializeEnd(indent);
}

string Serializable::serializeBegin(string indent) const
{
    return "{\n";
}

string Serializable::serializeElements(string indent) const
{
    string r;
    
    if( type != "" )
        r += TAB + indent + "\"type\" : " + stringRepr(type) + ",\n";
    if( name != "" )
        r += TAB + indent + "\"name\" : " + stringRepr(name) + ",\n";
    
    int size = properties.size();
    
    int i = 0;
    for(vector<Property>::const_iterator itr = properties.begin();
        itr!=properties.end();
        itr++ )
    {
        void* ptr = (char*)(this) + itr->offset;
        
        switch( itr->type )
        {
            case kString:
                r += TAB + indent + "\"" + itr->name + "\" : " + 
                    stringRepr(*(reinterpret_cast<string*>(ptr)));
            break;
            
            case kDouble:
                r += TAB + indent + "\"" + itr->name + "\" : " +
                    floatToString(*(reinterpret_cast<double*>(ptr)));
            break;
            
            case kInt:
                r += TAB + indent + "\"" + itr->name + "\" : " +
                    intToString(*(reinterpret_cast<int*>(ptr)));
            break;
            
            case kVectorDouble:
                {
                    r += TAB + indent + "\"" + itr->name + "\" : [\n";
                    vector<double>* vptr = (vector<double>*)ptr;
                    
                    vector<double>::iterator itr = vptr->begin();
                    if( itr != vptr->end() )
                    while(true)
                    {
                        r += TAB + TAB + indent + floatToString(*itr);
                        itr++;
                        if( itr==vptr->end() )
                        {
                            r += "\n";
                            break;
                        }
                        r += ",\n";
                    }
                    
                    r += TAB + indent + "]";
                }
            break;
            
            case kVectorInt:
                {
                    r += TAB + indent + "\"" + itr->name + "\" : [\n";
                    vector<int>* vptr = (vector<int>*)ptr;
                    
                    vector<int>::iterator itr = vptr->begin();
                    if( itr != vptr->end() )
                    while(true)
                    {
                        r += TAB + TAB + indent + intToString(*itr);
                        itr++;
                        if( itr==vptr->end() )
                        {
                            r += "\n";
                            break;
                        }
                        r += ",\n";
                    }
                    
                    r += TAB + indent + "]";
                }
            break;
            
            case kVectorString:
                {
                    r += TAB + indent + "\"" + itr->name + "\" : [\n";
                    vector<string>* vptr = (vector<string>*)ptr;
                    
                    vector<string>::iterator itr = vptr->begin();
                    if( itr != vptr->end() )
                    while(true)
                    {
                        r += TAB + TAB + indent + stringRepr(*itr);
                        itr++;
                        if( itr==vptr->end() )
                        {
                            r += "\n";
                            break;
                        }
                        r += ",\n";
                    }
                                        
                    r += TAB + indent + "]";
                }
            break;
            
            case kObject:
                r += TAB + indent + "\"" + itr->name + "\" : " +
                    (reinterpret_cast<Serializable*>(ptr))->serialize(indent + TAB);
            break;
        }
        
        if( i!=(size-1) )
            r += ",";
        r += "\n";
        
        i++;
    }
    
    return r;
}

string Serializable::serializeEnd(string indent) const
{
    return indent + "}";
}

void Serializable::addProperty(const std::string& name, Serializable& element)
{
    properties.push_back(Property(kObject, name, (char*)(&element) - (char*)(this) ));
}

void Serializable::addMember(const std::string& name, std::string& element)
{
    properties.push_back(Property(kString, name, (char*)(&element) - (char*)(this) ));
}

void Serializable::addMember(const std::string& name, double& element)
{
    properties.push_back(Property(kDouble, name, (char*)(&element) - (char*)(this) ));
}

void Serializable::addMember(const std::string& name, int& element)
{
    properties.push_back(Property(kInt, name, (char*)(&element) - (char*)(this) ));
}

void Serializable::addMember(const std::string& name, vector<double>& element)
{
    properties.push_back(Property(kVectorDouble, name, (char*)(&element) - (char*)(this) ));
}

void Serializable::addMember(const std::string& name, vector<int>& element)
{
    properties.push_back(Property(kVectorInt, name, (char*)(&element) - (char*)(this) ));
}

void Serializable::addMember(const std::string& name, vector<string>& element)
{
    properties.push_back(Property(kVectorString, name, (char*)(&element) - (char*)(this) ));
}


IntProperty::IntProperty() : value(0) {}
IntProperty::IntProperty(int i) : value(i) {}

int IntProperty::operator()() const
{
    return value;
}

void IntProperty::operator()(int i)
{
    value = i;
}

bool IntProperty::operator==(int i) const
{
    return value == i;
}

bool IntProperty::operator!=(int i) const
{
    return value != i;
}

IntProperty::operator int&()
{
    return value;
}

IntProperty::operator int const&() const
{
    return value;
}

string IntProperty::serialize(string indent) const
{
    return intToString(value);
}

void IntProperty::initWithParseNode(const parse::Node* n)
{
    value = n->data.i;
}


BoolProperty::BoolProperty() : value(false) {}
BoolProperty::BoolProperty(bool b) : value(b) {}

bool BoolProperty::operator()() const
{
    return value;
}

void BoolProperty::operator()(bool b)
{
    value = b;
}

string BoolProperty::serialize(string indent) const
{
    return value?"1":"0";
}

void BoolProperty::initWithParseNode(const parse::Node* n)
{
    value = n->data.i ? true : false;
}

BoolProperty::operator bool&()
{
    return value;
}

BoolProperty::operator bool() const
{
    return value;
}


DoubleProperty::DoubleProperty() : value(0) {}
DoubleProperty::DoubleProperty(double x) : value(x) {}

double DoubleProperty::operator()() const
{
    return value;
}

void DoubleProperty::operator()(double x)
{
    value = x;
}

bool DoubleProperty::operator==(double t) const
{
    return value == t;
}

bool DoubleProperty::operator!=(double t) const
{
    return value != t;
}

DoubleProperty::operator double&()
{
    return value;
}

DoubleProperty::operator double const&() const
{
    return value;
}

string DoubleProperty::serialize(string indent) const
{
    return floatToString(value);
}

void DoubleProperty::initWithParseNode(const parse::Node* n)
{
    value = n->data.x;
}


StringProperty::StringProperty() : value("") {}
StringProperty::StringProperty(const string& s) : value(s) {}

StringProperty::StringProperty(const char* s)
{
    value = s;
}

const string& StringProperty::operator()() const
{
    return value;
}

void StringProperty::operator()(const string& s)
{
    value = s;
}

string StringProperty::serialize(string indent) const
{
    return stringRepr(value);
}

void StringProperty::initWithParseNode(const parse::Node* n)
{
    value = n->data.s;
}


void Serializable::display() const
{
    g2clog( "%s\n", serialize().c_str() );
}


} // end namespace
