# Description des programmes
## Auteurs : Vincent Causse | Bastien Muraccioli | MEA4 2020-2021

- **Commande_robot_LED.py** Permet le déplacement du robot en fonction de la position d'un QR code en face de la caméra
    - ***Attention :*** La gestion du déplacement n'a pas été finalisée, la connexion MAVLink entre la Raspbery Pi (Companion computer) et la PixHawk (Ardusub) n'a pas totalement été configuré
    - Il se peut que le programme reste en attente d'un heartbeat (ping) qu'il ne reçoit pas. (https://www.ardusub.com/developers/pymavlink.html)
    

- **Commande_robot_LEDnoMavlink.py** Ce programme est une copie de Commande_robot_LED.py sans la liason Mavlink et donc sans l'actionnement des moteurs
    - Il permet de vérifier le fonctionnement de l'agortihme de détection de QRcode sur le Bluerov
    - Pour fonctionner il faut installer OpenCV comme décrit ici : https://www.ardusub.com/developers/opencv.html
    Ainsi que le module python Pyzbar, pour ce faire il faut mettre à jour sur la Raspberry Pi, python et pip avec `sudo apt update` et `sudo pip install upgrade`
    - Le programme va allumer des LEDs et afficher des messages dans la console en fonction des commandes qui devait être envoyé au robot.
    - L'utilisation des LEDs n'ont pas d'interet sur le Bluerov, nous les utilisions sur nos tests quand nos nous n'avions pas accès au robot


- **cmd_Raspberry.py** Permet d'utiliser les moteurs en fonction du QRcode, n'utilise pas de LEDs et affiche des messages dans la console pour vérifier l'envoi de commande.
    - ***Attention :*** Ici même problème au niveau de la connexion MAVLink qui n'a pas pu être testé (voir Commande_robot_LED.py)
    - Dans ce programme le flux vidéo est récupéré par Gstreamer qui permet que le flux soit utilisé en parallèle par ce programme et par QGroundControl
    - ***Attention 2 :*** Pour que le flux vidéo soit utilisé en parallèle, il faut aller ici : http://192.168.2.2:2770/camera 
    et changer `! udpsink host=192.168.2.1 port=5600` par `! multiudpsink clients=192.168.2.1:5600,192.168.2.1:4777` (Voir : https://www.ardusub.com/developers/opencv.html)
    - La caméra doit aussi être paramétré en 640x480 pour ce programme (résolution plus basse pour des opérations plus réactives, la puissance de la Raspberry Pi est limitée).

<a href="https://ibb.co/7X0sw3V"><img src="https://i.ibb.co/g3ck50w/diagramme-commande-robot.png" alt="diagramme-commande-robot" border="0" /></a>

- **QRcode_video.py** est le test de la détection du QRcode avec l'affichage du polygone, du rectangle et des points pour ordinateur

- **LedTest.py** est un test pour vérifier si les LEDs utilisées dans les programmes précédents sont correctement connectées