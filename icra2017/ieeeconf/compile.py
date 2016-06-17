#!/usr/bin/python

import subprocess, sys

commands = [
	['pdflatex', sys.argv[1] + '.tex'],
	]

for c in commands:
	subprocess.call(c)
