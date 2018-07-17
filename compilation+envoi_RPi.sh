#!/bin/bash

if [ -d /opt/tools/arm-bcm2708 ] && [ -d /usr/share/doc/python-pip ] && [ ! -d /usr/local/lib/python2.7/dist-packages/future ]; then	#si le repertoire arm-linux-gnueabihf et python-pip existent et si python future n existe pas
	echo "installation du paquet python future"
	sudo pip install future

elif [ -d /opt/tools/arm-bcm2708 ] && [ ! -d /usr/share/doc/python-pip ] ; then #si le repertoire arm-linux-gnueabihf existe et python-pip n existe pas
	echo "installation du paquet python future et de python-pip"
	sudo apt-get install python-pip
	sudo pip install future

elif [ ! -d /opt/tools/arm-bcm2708 ] && [ -d /usr/share/doc/python-pip ] && ! -d /usr/local/lib/python2.7/dist-packages/future ]; then 	#si le repertoire arm-linux-gnueabihf et python future n existent pas et si python-pip existe
	sudo git clone --depth 1 https://github.com/raspberrypi/tools.git /opt/tools
	sudo pip install future

elif [ ! -d /opt/tools/arm-bcm2708 ] && [ ! -d /usr/share/doc/python-pip ]; then #si le repertoire arm-linux-gnueabihf et python-pip n existent pas
	echo "installation du paquet python future et de python-pip"
	sudo git clone --depth 1 https://github.com/raspberrypi/tools.git /opt/tools
	sudo apt-get install python-pip
	sudo pip install future
	
else #si tous les repertoires existent
	echo "tous les répertoires sont installés : prêt à la compilation"
fi

# commande de compilation
export PATH=/opt/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin:$PATH
alias waf="$PWD/modules/waf/waf-light" 
./waf configure --board=navio2
echo "Compilation du fichier"
./waf --targets bin/arducopter-quad
echo ""
echo "Envoi du fichier vers la RPi"
echo "Mot de passe : raspberry"
scp build/navio2/bin/arducopter-quad pi@172.16.10.10:~/ardupilot/build/navio2/bin/
