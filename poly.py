
import re
import sys
import json

args = sys.argv

def get_original_spt(path):
	f = open(path, 'r')
	spt = json.loads(f.read())
	print spt 
	f.close()

def merge_dicts(a, b):
	D = {}
	for x in a:
		D[x] = a[x]
	for x in b:
		D[x] = b[x]
	return D

def go():
	if len(args) < 2:
		print "Takes the name of a file and generates json polygons"
		return
	filename = args[1]
	f = open(filename, 'r')
	lines = f.readlines()
	f.close()
	polygon = []
	for line in lines:
		l = line.split()
		if len(l) == 6:
			q = map(float, l)
			polygon.append([q[0], q[1]])
	print json.dumps(merge_dicts(original_sprite, {'collisionPolygon':polygon}))

go()

get_original_spt("hazel.world")


