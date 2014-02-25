
import sys

class Code:
	hcode = ''
	ccode = ''
	pairs = []
	
	def __init__(self):
		pass
	
	def make(self, world, classname):
		indent = '\t'
		self.make_pairs(world)
		self.begin_class(classname)
		self.declare_variables(indent)
		self.end_class(classname)
		self.begin_cfile(classname)
		self.begin_constructor(classname)
		self.initialize_variables(indent)
		self.end_constructor(classname)
		self.empty_destructor(classname)
		self.begin_init(classname)
		self.get_nodes(indent)
		self.end_init(classname)
	
	def make_pairs(self, node):
		self.pairs += [('g2c::' + node['type'], node['name'])];
		if node.has_key('children'):
			for n in node['children']:
				self.make_pairs(n)
	
	def begin_class(self, classname):
		self.hcode += '\n#ifndef _' + classname.upper() + '_\n'
		self.hcode += '#define _' + classname.upper() + '_\n\n'
		self.hcode += '#include "sprites.h"\n\n'
		self.hcode += 'using namespace g2c;\n'
		self.hcode += 'using namespace std;\n'
		self.hcode += 'class ' + classname + ' : public g2c::World {\n'
		self.hcode += 'public:\n'
		self.hcode += '\t' + classname + '();\n'
		self.hcode += '\tvirtual ~' + classname + '();\n\n'
	
	def end_class(self, classname):
		self.hcode += '\n'
		self.hcode += '\tvoid init();\n'
		self.hcode += '\tvirtual Node* getNode(const string& name) = 0;\n'
		self.hcode += '};\n'
		self.hcode += '#endif\n'
	
	def begin_cfile(self, classname):
		self.ccode += '\n#include "' + classname.lower() + '.h"\n\n'
	
	def begin_constructor(self, classname):
		self.ccode += '\n'
		self.ccode += classname + '::' + classname + '() :\n'
	
	def initialize_variables(self, indent):
		l = []
		for type,name in self.pairs:
			l += [indent + name + '(NULL)']
		self.ccode += ',\n'.join(l) + "\n"
	
	def end_constructor(self, classname):
		self.ccode += '{\n}\n\n'
	
	def empty_destructor(self, classname):
		self.ccode += classname + '::~' + classname + '()\n{\n}\n\n'
	
	def declare_variables(self, indent):
		for type,name in self.pairs:
			self.hcode += indent + type + '* ' + name + ';\n'
	
	def begin_init(self, classname):
		self.ccode += 'void ' + classname + '::init()\n'
		self.ccode += '{\n'
	
	def get_nodes(self, indent):
		for type,name in self.pairs:
			self.ccode += indent + name + ' = (' + type + '*)(getNode("' + name + '"));\n'
	
	def end_init(self, classname):
		self.ccode += '}\n'


topoffile = """
/****************************************************/
/*         AUTO GENERATED CODE, DO NOT EDIT         */
/****************************************************/

"""


if len(sys.argv) < 3:
	print "takes the name of a .spt file and the name of a class."
	print "Generates a .cpp file and .h file of the same name as"
	print "the class."
else:
	sptfile, classname = sys.argv[1:]
	
	print "running basegen.py..."
	
	f = open(sptfile, 'r')
	s = f.read()
	f.close()
	
	world = eval(s)
	
	code = Code()
	code.make(world, classname)
	
	f = open(classname.lower() + '.h', 'w')
	f.write(topoffile + code.hcode)
	f.close()
	
	f = open(classname.lower() + '.cpp', 'w')
	f.write(topoffile + code.ccode)
	f.close()
	
	print "...done"

