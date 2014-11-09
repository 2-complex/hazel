
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#if( !defined(WINDOWS) )
#include <sys/time.h>
#endif

#include <string>
#include <math.h>
#include <time.h>

#include "util.h"

namespace g2c {




double currentTime()
{
#if( defined(WINDOWS) )
    return std::clock();
#else
    struct timeval t;
    gettimeofday(&t, NULL);
    return 0.000001 * t.tv_usec + 1.0 * t.tv_sec;
#endif
}

double clamp(double x, double m, double M)
{
    if( x > M )
        x = M;
    
    if( x < m )
        x = m;
    
    return x;
}

double min(double x, double y)
{
    return (x < y) ? x : y;
}

double max(double x, double y)
{
    return (x > y) ? x : y;
}

double myfmod(double x, double m)
{
    return fmod(fmod( x, m ) + m, m);
}

double sCurve(double x)
{
    if(x>1.0) return 1.0;
    if(x<0.0) return 0.0;
    return 3*x*x-2*x*x*x;
}

unsigned char floatToByte(double t)
{
    int n = (int)(t*256.0);
    if(n<0)n=0;
    if(n>255)n=255;
    return (unsigned char)n;
}

std::string floatToString(double t)
{
    char buffer[255];
    sprintf(buffer, "%.9f", t);
    for(int i = strlen(buffer)-1; i>=0; i--)
    {
        char c = buffer[i];
        if(c=='0' || c=='.')
        {
            buffer[i] = 0;
            if(c=='.')
                break;
        }
        else
            break;
    }
    return std::string(buffer);
}

std::string intToString(int n)
{
    char r[255];
    sprintf(r, "%d", n);
    return std::string(r);
}

int stringToInt(const std::string& s)
{
    return atoi(s.c_str());
}

double stringToFloat(const std::string& s)
{
    return atof(s.c_str());
}

std::string stringRepr(const std::string& s)
{
    std::string r = "\"";
    for(int i = 0; s[i]; i++)
    {
        char c = s[i];
        if( c == '\\' )
            r += "\\\\";
        else if( c == '"' )
            r += "\\\"";
        else if( c == '\n' )
            r += "\\n";
        else
            r += c;
    }
    r += "\"";
    return r;
}

std::string intToStringWithCommas(int n)
{
    bool negative = (n < 0);
    
    if( negative )
        n = -n;
    
    char r[255];
    sprintf(r, "%d", n);
    int len = strlen(r);
    
    if( len > 3 )
    {
        int gap = (len-1) / 3;
        int j = len-1+gap;
        for(int i = len-1; i>=0; i--)
        {
            r[j--] = r[i];
            if((len-i)%3==0 && j>=0)
                r[j--] = ',';
        }
        r[len+gap] = 0;
    }
    
    return (negative ? "-" : "") + std::string(r);
}

bool charInString(char c, const char* s)
{
    for(int i = 0; s[i]; i++)
        if( c == s[i] )
            return true;
    return false;
}

std::string flattenWhitespace(const char* s)
{    
    int n = strlen(s);
    char* r = new char[strlen(s)+1];
    
    char quotebegin = 0;
    bool escaped = false;
    
    int j = 0;
    for(int i = 0; i < n; i++)
    {
        char c = s[i];
        if( c == '\\' )
            escaped = true;
        if( !escaped && charInString(c, "\'\"") )
        {
            if(quotebegin)
                quotebegin = 0;
            else
                quotebegin = c;
        }
        if( !escaped && !quotebegin && charInString(c, "\t\n\t ") )
        {
            if( j != 0 && r[j-1]!=' ' )
                r[j++] = ' ';
        }
        else
            r[j++] = c;
    }
    r[j++] = 0;
    
    std::string result(r);
    delete r;
    
    return result;
}

void seedRand()
{
    int seed = 0xc0ffee;

#if( !defined(WINDOWS) )
    struct timeval tv;
    gettimeofday(&tv, NULL);
    srandom(0xc0ffee + tv.tv_usec + tv.tv_sec);
#endif

    srand( seed );
}

int randInt(int m)
{
    return ((rand() % m) + m) % m;
}

std::string directoryOfPath(const std::string& path)
{
    int i = path.size()-1;
    for(; i>=0; i--)
    {
        if(path[i] == '/')
            break;
    }
    return path.substr(0, i+1);
}

std::string fileOfPath(const std::string& path)
{
    int n = path.size();
    int i = n-1;
    for(; i>=0; i--)
    {
        if(path[i] == '/')
            break;
    }
    i++;
    return path.substr(i, n-i);
}

std::vector<std::string> components(const std::string& path)
{
    std::vector<std::string> result;
    
    std::string s;
    
    int n = path.size();
    for(int i = 0; i < n; i++)
    {
        char c = path[i];
        if( c == '/' )
        {
            if( s.size() )
                result.push_back(s);
            s.clear();
        }
        else
            s += c;
    }
    
    if( s.size() )
        result.push_back(s);
    
    return result;
}

std::string relativePath(const std::string& source, const std::string& target)
{
    std::string result;
    
    std::vector<std::string>
        source_components = components(source),
        target_components = components(target);
    
    int m = source_components.size(),
        n = target_components.size();
    
    int first = 0; // index of the first entries that differ.
    for( ; first < m; first++ )
        if( source_components[first] != target_components[first] )
            break;
    
    for( int i = 0; i < m - first - 1; i++ )
        result += "../";
    
    for( int i = first; i < n; i++ )
        result += (i==first?"":"/") + target_components[i];
    
    return result;
}

bool endsWith(const std::string& path, const std::string& extension)
{
    int n = path.size();
    int m = extension.size();
    
    if( n < m )
        return false;
    
    for( int i = 0; i < m; i++ )
    {
        if( path[n-1-i] != extension[m-1-i] )
            return false;
    }
    return true;
}

} // end namespace

