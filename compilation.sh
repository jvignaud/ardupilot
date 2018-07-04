#!/bin/bash

if [-d /opt/tools/arm-bcm2708]		#si le répertoire arm-linux-gnueabihf existe
	if [-d /usr/share/doc/python-pip]	#si le répertoire python-pip existe
		if [!-d /usr/local/lib/python2.7/dist-packages/future]	#si le répertoire future n'existe pas
			sudo pip install future
		fi
	else					#si le répertoire python-pip n'existe pas
		sudo apt-get install python-pip
		sudo pip install future
	fi
else 					#si le répertoire arm-linux-gnueabihf n'existe pas
	if [-d /usr/share/doc/python-pip]	#si le répertoire python-pip existe
		if [!-d /usr/local/lib/python2.7/dist-packages/future]	#si le répertoire future n'existe pas
			sudo pip install future
		fi
	else	#si le répertoire python-pip n'existe pas
		sudo apt-get install python-pip
		sudo pip install future
fi

# commande de compilation
export PATH=/opt/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin:$PATH
alias waf="$PWD/modules/waf/waf-light" 
./waf configure --board=navio2
./waf --targets bin/arducopter-quad
