import subprocess
import os
import numpy as np
import sys

robots = [0,1,2]
limit = 6000
delay = 2

protoarch = " ./proto_arch "
echo = " echo "

experiment_name = "exp2_2_man"				#mo bf eo
dummybot = "-mo 1.0 -delay 2 -mfol -1 -ofol -1 -mow 0.9548 0.5 0.5 -beh 0"
dummyname = "_-1.0_0.9_0.5_0.5"
testbot = "-mo -1.0 -delay 2 -mfol -1 -ofol -1 -mow 0.9548 0.5 0.5 -beh 0"
testname = "_-1.0_0.9_0.5_0.5"

i = str(sys.argv[1])

for robot in robots:
	# prepare everything
	preamble = "gnome-terminal --hide-menubar -t \"ROBOT "+str(robot)+"\" -x"
	if(robot == 0):
		args="-idx "+str(robot)+" "+testbot+" -limit "+str(limit)+" "
		outfile = "-csv \""+experiment_name+"/out_robot"+str(robot)+testname+"_iter"+str(i)+".txt\""
	else:
		args="-idx "+str(robot)+" "+dummybot+" -limit "+str(limit)+" "
		outfile = "-csv \""+experiment_name+"/out_robot"+str(robot)+dummyname+"_iter"+str(i)+".txt\""					
	command = protoarch+args+outfile			
	print preamble+command
	# start the controllers
	if(robot == robots[len(robots)-1]): 
		os.system(command)
	else:
		subprocess.Popen(command,shell=True)
