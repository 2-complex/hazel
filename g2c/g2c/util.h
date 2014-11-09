
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

#ifndef _UTIL_
#define _UTIL_

#include <string>
#include <vector>

namespace g2c {

/*! Returns the current time since the epoch, in seconds, as a double.*/
double currentTime();

/*! If x is bigger than M returns M, if x is smaller than m, returns m, otherwise returns x.*/
double clamp(double x, double m, double M);

/*! Returns whichever of x and y is smallest.*/
double min(double x, double y);

/*! Returns whichever of x and y is greatest.*/
double max(double x, double y);

/*! Returns a number in the interval [0, m).*/
double myfmod(double x, double m);

/*! If x < 0 returns 0, if x > 1 returns 1, otherwise uses a cubic funciton of x to get a smooth change between x 0 and 1. */
double sCurve(double x);

/*! */
unsigned char floatToByte(double t);

/*! Returns a string representation of the number t.  For instance floatToString(3.1) returns "3.1".*/
std::string floatToString(double t);

/*! Takes a string representing an integer and returns that integer, for instance stringToInt("-3") returns -3. */
int stringToInt(const std::string& s);

/*! Takes a string representing a number and returns a double.  For instance stringToFloat("1.1") = 1.1.*/
double stringToFloat(const std::string& s);

/*! Takes an int as an argument and returns a string representation of that int.  For instance intToString(-3) return "-3".*/
std::string intToString(int n);

/*! Takes an int as an argument and returns a string representation of that int with commas serarating tiples of digits.
    For instance intToString(1000) return "1,000".*/
std::string intToStringWithCommas(int n);

/*! Takes a string as an argument, returns a string literal which evaulates to that string surrounded by quotation marks.*/
std::string stringRepr(const std::string& s);

/*! Returns true of the given character is in the given (ASCII) string.*/
bool charInString(char c, const char* s);

/*! Remove unnecessary whitespace from a json string.*/
std::string flattenWhitespace(const char* s);

/*! Seeds the random number generator with the clock.*/
void seedRand();

/*! Returns a random int in the range 0...(m-1).*/
int randInt(int m);

/*! Takes a path as an argument and returns a path to the directory containing that path.*/
std::string directoryOfPath(const std::string& path);

/*! Takes a path to a file and returns just the name of the file.*/
std::string fileOfPath(const std::string& path);

/*! Takes two paths and returns a relative path between them.*/
std::string relativePath(const std::string& source, const std::string& target);

/*! Separates a path into path components.  For instance components("a/b/c") would return a vector containing strings "a", "b" and "c".*/
std::vector<std::string> components(const std::string& path);

/*! Returns true if the string ends with the given suffix.*/
bool endsWith(const std::string& path, const std::string& extension);

} // end namespace

#endif


