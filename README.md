# Drone 

## ENGLISH VERSION 

-> README.ENG.md

## Pourquoi ce projet ?

1) Je voulais faire un projet sur un système que je trouve plus intéressant et épanouissant qu'un simple bras vu et revu
2) À défaut de paraitre cliché, construire un objet volant procure une certaine joie, "la conquête du ciel"
3) J'ai toujours voulu piloter un drone aussi simple que ça

## Description

Le but du projet (au-delà de me faire plaisir) est aussi de pouvoir utiliser mes compétences dans un cas concret, ainsi que d'en développer de nouvelles. 
C'est pourquoi j'ai fait le plan suivant :

### À FAIRE

- Modéliser la dynamique
- Construire le corps du drone
- Construire mon "Flight Controller" (dans le sens PCB)
- Construire mon Contrôleur (dans le sens PID)
- Coder l'OS (en utilisant l'utilitaire STM)


### À NE PAS FAIRE 
- La plaque de distribution d'alimentation (PDB)
- ESC 
- Construire l'antenne
- Moteurs
- Hélices

Ce que j'ai décidé de ne pas faire (== prendre les composants déjà faits) est pour des raisons de temps ainsi que financières (acheter un pdb produits en grande quantité revient ~5 fois moins cher que le faire soit même). Le FC étant le cerveau du drone je trouvais ça pertinent de le faire moi même.

## Progression 

- Modéliser la dynamique ✔
- Construire le corps du drone ✔
- Construire mon "Flight Controller" (dans le sens PCB) ✔
- Construire mon Contrôleur (dans le sens PID) ✔
- Coder l'OS (en utilisant l'utilitaire STM) ❌ (Manque quelques finitions : arrêt d'urgence, le mixer, mahony)
- La plaque de distribution d'alimentation (PDB) ✔
- ESC ✔
- Construire l'antenne ✔

Actuellement, le contrôleur est fait au sens théorique, il est codé, mais il me reste à tuner les gains des correcteurs. J'attends de remplacer un des moteurs qui est défaillant pour pouvoir ensuite faire les tests.

## Les fichiers

- Pour les équations/explications/quelques images -> drone.pdf 
- Pour le code -> FlightController
- Pour les fichiers gerber et la CAO du drone -> me demander par mail : **benjamin@93160@gmail.com**

## Quelques mots clefs :

- Vue d'ensemble : Quadrotor, CAO (Fusion360), PCB (EasyEDA), STM32 (CubeMX & IDE), ESP32 (Arduino), C, Contrôleur en cascade;
- PCB : SPI, I2C, DMA, UART, USB;

