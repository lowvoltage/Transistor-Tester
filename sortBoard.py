#!/usr/bin/env python

import string
import sys
import shutil

fin = open(sys.argv[1],'r')
fout = open('tmp.brd', 'w')

tracks = False
trackPairs = []

for line in fin.readlines(): 
	if line.startswith('$TRACK'):
		tracks = True
		fout.write(line)
	elif line.startswith('$EndTRACK'):
		trackPairs.sort()
		for pair in trackPairs:
			fout.write(pair)
		tracks = False
		fout.write(line)
	elif tracks:
		if line.startswith('Po'):
			trackPairs.append(line)
		else:
			i = len(trackPairs) - 1
			trackPairs[i] = trackPairs[i] + line
	else:
		fout.write(line)

fin.close()
fout.close()

shutil.copy(sys.argv[1], sys.argv[1] + '.backup')
shutil.move('tmp.brd', sys.argv[1])
