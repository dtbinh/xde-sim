import re
import sys
import os.path

names = ["qJac", "dqJac", "ddqJac", "absdqJac", "absddqJac", "distRObsJac"]

for name in names:

	idx = 1

	print "\033[1;31m" + name + "\033[0m"

	while os.path.isfile("/home/mendesfilho/Dev/" + name + str(idx) + ".txt"):


		f = open("/home/mendesfilho/Dev/" + name + str(idx) + ".txt", "r")
		eqCons = f.read()

		def idxrplcer(matchobj):
			nIdx = str(int(matchobj.group(1))-1)
			return "X["+nIdx+"]"

		eqCons = re.sub(r"X1_([0-9]*)", idxrplcer, eqCons)

		f.close()
		f = open("/home/mendesfilho/Dev/jacobs/" + name + str(idx) + ".cpp", "w")
		f.write(eqCons)

		print "/home/mendesfilho/Dev/jacobs/" + name + str(idx) + ".cpp"

		idx+=1