import subprocess
import os
import numpy as np

repeats = 50
robots = [0,1,2]
limit = 6000
delay = 2

protoarch = " ./proto_arch "
echo = " echo "

experiment_name = "exp1_1"				#mo bf eo
dummybot = "-mo -1.0 -delay 2 -mfol -2 -ofol -1 -mow 0.9 1.0 0.0 "
dummyname = "_-1.0_0.9_1.0_0.0"
testbot = dummybot
testname = dummyname

for i in range(repeats):
	# start the simulator
	subprocess.Popen("./sendCmd -start 1",shell=True)	
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
	# stop the simulator
	os.popen("./sendCmd -stop 0")
	os.popen("sleep 1")
