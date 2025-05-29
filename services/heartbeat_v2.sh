#!/bin/bash
#
# Gère la LED1 / GPIO20 (PIN38) => clignote, 0.5 HZ
#  + si BTN_2 / GPIO25 (PIN 22) enfoncé, alors redémarre le service robot_strat
#
# AV, 2025-05-27
#

# Exécuter en root !    NOP -> sinon le "systemctl --user..." défaille
#if [ $UID != 0 ]; then
#	sudo $0
#	wait
#	exit 0
#fi


LED1_VAL=0
while true ; do
	# Fait clignoter LED1
	gpioset 4 20=$LED1_VAL
	LED1_VAL=$(( ( LED1_VAL + 1) % 2 ))

	# Redémarre robot_strat qd BTN_2 activé
       	BTN_2=$( gpioget -l --bias=pull-up 4 25 )
	echo Etat bouton 2 = $BTN_2 # debug
	[ $BTN_2 == 1 ] && systemctl --user restart robot_strat.service && date && echo Service STRAT redémarré
	sleep 1
done
