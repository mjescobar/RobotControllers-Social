#! /bin/bash
# usage: ./proto_arch -idx <modi id> -mo <initial MO> -delay <time before start> -beh <selected behavior> -mfol <modi to follow> -ofol <object to follow> -mow <w_mo> <w_bf> <w_eo>
 COUNTER=0
 while [  $COUNTER -lt 10 ]; do
	 echo The counter is $COUNTER
	 COUNTER=$((COUNTER+0.5))
 done
#gnome-terminal --hide-menubar -t "ROBOT 3" -x ./proto_arch -idx 3 -mo -1.0 -delay 4 -mfol  0 -ofol 0 -mow 0.992 0.5 0.5 -beh 0 -limit 5000
#gnome-terminal --hide-menubar -t "ROBOT 4" -x ./proto_arch -idx 4 -mo -1.0 -delay 4 -mfol  0 -ofol 0 -mow 0.992 0.5 0.5 -beh 0 -limit 5000
