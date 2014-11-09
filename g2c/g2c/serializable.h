
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


#ifndef _SERIALIZABLE_
#define _SERIALIZABLE_

#include "parse.h"

#include <vector>
#include <string>

const std::string TAB("  ");

namespace g2c {


/*! Serializable has methods serialize() which returns a string representation of the object
    and deserialize(const string&) which takes a string representation as an argument and populates
    the members to match the representation.  Subclasses can override various functions to control
    the output of serialize() and the interpretation in deserialize().  There are a few ways of doing
    this.  One way is to override serialize() and deserialize() directly.  However doing this
    requires that the subclass own the entire job of serializing, there are other virtual functions
    that require less work.*/
class Serializable {
    enum PropertyType {
        kInt,
        kDouble,
        kString,
        kObject,
        kVectorDouble,
        kVectorInt,
        kVectorString
    };
    
    class Property {
    public:
        Property(PropertyType type, const std::string& name, long offset);
        PropertyType type;
        std::string name;
        long offset;
    };
    
public:
    std::string type;
    std::string name;
    
    virtual std::string serialize(std::string indent = "") const;
    virtual void deserialize(const std::string& s);
    virtual void initWithParseNode(const parse::Node* n);
    
    std::string getType(const parse::Node* n) const;
    virtual void handleChild(const parse::Node* n);
    
    virtual void display() const;
    
protected:
    virtual std::string serializeBegin(std::string indent = "") const;
    virtual std::string serializeElements(std::string indent = "") const;
    virtual std::string serializeEnd(std::string indent = "") const;
    
    void addProperty(const std::string& name, Serializable& element);
    
    void addMember(const std::string& name, std::string& element);
    void addMember(const std::string& name, double& element);
    void addMember(const std::string& name, int& element);
    void addMember(const std::string& name, std::vector<double>& element);
    void addMember(const std::string& name, std::vector<int>& element);
    void addMember(const std::string& name, std::vector<std::string>& element);
    
private:
    std::vector<Property> properties;
};

class IntProperty : public Serializable {
public:
    IntProperty();
    IntProperty(int i);
    
    virtual std::string serialize(std::string indent = "") const;
    virtual void initWithParseNode(const parse::Node* n);
    
    int operator()() const;
    void operator()(int i);
    
    bool operator==(int i) const;
    bool operator!=(int i) const;
    operator int&();
    operator int const&() const;
    
private:
    int value;
};

class BoolProperty : public Serializable {
public:
    BoolProperty();
    BoolProperty(bool b);
    
    virtual std::string serialize(std::string indent = "") const;
    virtual void initWithParseNode(const parse::Node* n);
    
    bool operator()() const;
    void operator()(bool b);
    operator bool&();
    operator bool() const;

private:
    bool value;
};

class DoubleProperty : public Serializable {
public:
    DoubleProperty();
    DoubleProperty(double x);
    
    virtual std::string serialize(std::string indent = "") const;
    virtual void initWithParseNode(const parse::Node* n);
    
    bool operator==(double t) const;
    bool operator!=(double t) const;
    double operator()() const;
    void operator()(double x);
    operator double&();
    operator double const&() const;

private:
    double value;
};

class StringProperty : public Serializable {
public:
    StringProperty();
    StringProperty(const std::string& s);
    StringProperty(const char* s);
    
    virtual std::string serialize(std::string indent = "") const;
    virtual void initWithParseNode(const parse::Node* n);
    
    const std::string&  operator()() const;
    void operator()(const std::string& s);

private:
    std::string value;
};

template<class T>
class VectorProperty : public Serializable,
                       public std::vector<T> {
public:
    VectorProperty<T>() {}
    VectorProperty<T>(const std::vector<T>& v) : std::vector<T>(v) {}
    
    virtual std::string serialize(std::string indent = "") const
    {
        std::string r = "[\n";
        
        int n = std::vector<T>::size();
        
        for(int i = 0; i < n; i++)
        {
            r += TAB + indent + std::vector<T>::at(i).serialize(indent + TAB) +
                std::string((i==n-1)?"":",") + "\n";
        }
        
        r += indent + "]";
        
        return r;
    }
    
    void initWithParseNode(const parse::Node* n)
    {
        std::vector<T>::clear();
        for(std::vector<parse::Node*>::const_iterator itr = n->children.begin();
            itr!=n->children.end();
            itr++)
        {
            T t;
            t.initWithParseNode(*itr);
            std::vector<T>::push_back(t);
        }
    }
};


template<class T>
class PointerVectorProperty : public Serializable,
                              public std::vector<T> {
public:
    PointerVectorProperty<T>() {}
    PointerVectorProperty<T>(const std::vector<T>& v) : std::vector<T>(v) {}
    
    virtual std::string serialize(std::string indent = "") const
    {
        std::string r = "[\n";
        
        int n = std::vector<T>::size();
        
        for(int i = 0; i < n; i++)
        {    
            r += TAB + indent + std::vector<T>::at(i)->serialize(indent + TAB) +
                std::string((i==n-1)?"":",") + "\n";
        }
        
        r += indent + "]";
        
        return r;
    }
    
    void initWithParseNode(const parse::Node* n)
    {
    }
};

} // end namespace


#endif
