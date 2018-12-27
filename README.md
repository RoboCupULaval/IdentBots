# Identification

## RobotIdent
Interface permettant l'acquisition de données directement des robots via un port série Bluetooth.

## Modelisation
...


## Why-do, What-do, How-do.
Si tu es ici, c'est parce que la loi de contrôle de ton robot n'est peut-être plus à jour (architecture du robot a été modifiée par exemple).
En gros, ce que tu veux c'est les deux matrice M1 et M2 qui sont directement programées dans le microcontroleur du robot.
Ces matrices permettent de transférer des commandes de vitesse cartesiennes en vitesse de roues en prennant en compte plein plein de cossins le fun comme la friction et le back EMF (voir le paper sur le MNRC for more funny details).

Pour avoir ces deux matrices, il faut simplement aller dans IdentBots/Identification et faire rouler le script matlab identification_vehicule_madel_2.m
(warning, vu que l'architecture du robot a changer, il faut des fichers (contenu dans IdentBots/Identification/data/gamma, par exemple, pour caratériser la réponse en boucle ouverte. En date d'aujourd'hui, l'information pour les générer n'est pas encore bien propagée, ce tuto sera mis a jour pour ce détail sous peu.)
Les deux matrices seront disponibles dans le workspace de matlab.
CEPENDANT, il ne faut pas oublier de modifier les paramètres du robots en fonction des changement de l'architecture du robot!
ex: rayon des roues, distance des roues par rapport au centre du robot, ration de reduction (gear-ratio), signe astrologique du robot, etc.
Ensuite, pour implémenter ces matrices, il faut aller dans le git repro RobotMCU et modifier les deux matrices contenues dans RobotMCU/Robot_stm32f4discovery/Src/robocup/motors/mnrc.c
ATTENTION! la matrice M1 doit être inversée avant d'être retranscrite. Faire ceci directement dans matlab est pas mal plus simple.
Si on utilise la commande inv(M1) pour le faire, un message d'avertissement risque de pop⁻up parce que la matrice est presque singulière.
C'est normal puisqu'on a un robot a 4 roues (1 roue redondante pour les déplacement x-y+rotation).
Il faut alors utiliser la fonction pinv(M1) pour obtenir le bon résultat (chercher pseudo-inverse sur google si tu veux t'enfoncer une coche plus profond dans le terrier du lapin).


