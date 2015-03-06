import subprocess
import os
import numpy as np

#gnome-terminal --hide-menubar -t "ROBOT 4" -x ./proto_arch -idx 4 -mo -1.0 -delay 4 -mfol  0 -ofol 0 -mow 0.992 0.5 0.5 -beh 0 -limit 5000
repeats = 50
robots = [0,1,2]
mos = [-1.0,1.0]
w_mos = [0.9,0.992] #,0.995]
w_bfs = np.linspace(0.0,1,5);
w_eos = np.linspace(0.0,1,5);
limit = 6000
delay = 2

modi2follow = -1
obj2follow = -1

protoarch = " ./proto_arch "
echo = " echo "

#experiment_name = "all_the_same"
#experiment_name = "dummy_bots_v3" #"dummy_bots"
#dummybot = "-mo -1.0 -delay 2 -mfol  0 -ofol 0 -mow 0.992 0.5 0.5 "
#dummyname = "_-1.0_0.992_0.5_0.5"

#experiment_name = "dummy_bots_v4"
#dummybot = "-mo -1.0 -delay 2 -mfol  0 -ofol 0 -mow 0.7 0.5 0.5 "
#dummyname = "_-1.0_0.7_0.5_0.5"

experiment_name = "param"
dummybot = "-mo -1.0 -delay 2 -mfol  -2 -ofol -1 -mow 0.9 0.5 0.5 "
dummyname = "_-1.0_0.9_0.5_0.5"

for mo in mos:
	for w_mo in w_mos:
		for w_bf in w_bfs:
			for w_eo in w_eos: 
				for i in range(repeats):
					# start the simulator
					subprocess.Popen("./sendCmd -start 1",shell=True)	
					for robot in robots:
						# prepare everything
						preamble = "gnome-terminal --hide-menubar -t \"ROBOT "+str(robot)+"\" -x"
						if(robot == 0):
							modi2follow=-1
							args="-idx "+str(robot)+" -mo "+str(mo)+" -delay "+str(delay)+" -mfol  "+str(modi2follow)+" -ofol "+str(obj2follow)+" -mow "+str(w_mo)+" "+str(w_bf)+" "+str(w_eo)+" -limit "+str(limit)+" "
							outfile = "-csv \""+experiment_name+"/out_robot"+str(robot)+"_"+str(mo)+"_"+str(w_mo)+"_"+str(w_bf)+"_"+str(w_eo)+"_iter"+str(i)+".txt\""
						else:
							modi2follow=0	
							#args="-idx "+str(robot)+" -mo "+str(mo)+" -delay "+str(delay)+" -mfol  "+str(modi2follow)+" -ofol "+str(obj2follow)+" -mow "+str(w_mo)+" "+str(w_bf)+" "+str(w_eo)+" -limit "+str(limit)+" "
							#outfile = "-csv \""+experiment_name+"/out_robot"+str(robot)+"_"+str(mo)+"_"+str(w_mo)+"_"+str(w_bf)+"_"+str(w_eo)+"_iter"+str(i)+".txt\""
							args="-idx "+str(robot)+" "+dummybot+" -limit "+str(limit)+" "
							outfile = "-csv \""+experiment_name+"/out_robot"+str(robot)+dummyname+"_iter"+str(i)+".txt\""
						#execbeh = "-beh "+str(beh2exec)
						
						if(robot==0):
							command = protoarch+args+outfile
						else:
							#command = echo+args+execbeh+outfile
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
