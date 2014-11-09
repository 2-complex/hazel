
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


#ifndef _PARSE_
#define _PARSE_

#include "log.h"

#include <vector>
#include <string>

#include <string.h>

namespace parse {

class Node;

class Data {
public:
    Data() : i(0), x(0.0), value(NULL) {}
    virtual ~Data() {}
    int i;
    double x;
    std::string s;
    Node* value;
};

enum Type {
    kNode = 0,
    kInt = 1,
    kFloat,
    kString,
    kObject,
    kList,
    kTuple
};

class Node {
public:
    Node() :
        s(NULL),
        begin(0),
        end(0),
        type(kNode) {s = (char*)"";}
    
    Node(const char* s) : s(s),
        type(kNode) {begin = 0; end = strlen(s); parse();}
    
    Node(const char* s, int begin, int end) : 
        s(s),
        begin(begin),
        end(end),
        type(kNode) {parse();}
    
    virtual ~Node();
    
    const char* s;
    int begin;
    int end;
    
    Type type;
    Data data;
    
    std::vector<Node*> children;
    
    void display() const;
    void print() const;
    
protected:
    void parse();
    void clearChildren();
};

bool charInString(char c, const char* s);
    
}

#endif

