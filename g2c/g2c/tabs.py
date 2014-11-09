
import os
import re

def hasext(name, e):
	return name[len(name)-len(e):] == e

paths = []

for subdir, dirs, files in os.walk('./'):
	for name in files:
		if hasext(name, ".cpp") or hasext(name, ".h"):
			if subdir == "./":
				paths.append(name)
			else:
				paths.append(subdir + '/' + name)

for p in paths:
	f = open(p, "r")
	s = f.read()
	f.close()
	
	s = s.replace("\t", "    ")
	
	f = open(p, "w")
	f.write(s)
	f.close()
