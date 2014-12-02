#! /bin/bash
# usage: ./proto_arch -idx <modi id> -mo <initial MO> -delay <time before start> -beh <selected behavior> -mfol <modi to follow> -ofol <object to follow> -mow <w_mo> <w_bf> <w_eo>
gnome-terminal --hide-menubar -t "ROBOT 1" -x ./proto_arch -idx 1 -mo -1.0 -delay 4 -mfol  0 -ofol 0 -mow 0.992 0.5 0.5 -beh 0 -limit 5000
gnome-terminal --hide-menubar -t "ROBOT 2" -x ./proto_arch -idx 2 -mo -1.0 -delay 4 -mfol  0 -ofol 0 -mow 0.992 0.5 0.5 -beh 0 -limit 5000
beep -f 400 -l 500
sleep 2
./proto_arch -idx 0 -mo 1.0 -delay 4 -mfol -1 -ofol 0 -mow 0.992 0.3 0.7 -limit 5000 -csv "out0_00.txt"
beep -f 400 -l 500
sleep 2
./proto_arch -idx 0 -mo 1.0 -delay 4 -mfol -1 -ofol 0 -mow 0.992 0.0 1.0 -limit 5000 -csv "out0_01.txt"
beep -f 400 -l 500
sleep 2
./proto_arch -idx 0 -mo 1.0 -delay 4 -mfol -1 -ofol 0 -mow 0.992 1.0 0.0 -limit 5000 -csv "out0_02.txt"
beep -f 400 -l 500
sleep 2
./proto_arch -idx 0 -mo -1.0 -delay 4 -mfol -1 -ofol 0 -mow 0.992 0.3 0.7 -limit 5000 -csv "out0_03.txt"
beep -f 400 -l 500
sleep 2
./proto_arch -idx 0 -mo -1.0 -delay 4 -mfol -1 -ofol 0 -mow 0.992 0.0 1.0 -limit 5000 -csv "out0_04.txt"
beep -f 400 -l 500
sleep 2
./proto_arch -idx 0 -mo -1.0 -delay 4 -mfol -1 -ofol 0 -mow 0.992 1.0 0.0 -limit 5000 -csv "out0_05.txt"
beep -f 400 -l 500
sleep 2

#gnome-terminal --hide-menubar -t "ROBOT 3" -x ./proto_arch -idx 3 -mo -1.0 -delay 4 -mfol  0 -ofol 0 -mow 0.992 0.5 0.5 -beh 0 -limit 5000
#gnome-terminal --hide-menubar -t "ROBOT 4" -x ./proto_arch -idx 4 -mo -1.0 -delay 4 -mfol  0 -ofol 0 -mow 0.992 0.5 0.5 -beh 0 -limit 5000
