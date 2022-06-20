TODO
====
- Installation capteur de pluie
- Buzzer (Sortie PWM)
- LED d'état à l'intérieur (Allumée: télescope parqué, clignotante: arrêt d'urgence)
- Ajout clignotement lent pendant déplacement abri
- Ajout animation LEDs extérieur pendant attente (moteurOK, ouvre/ferme portes)
- Inversion de l'état "PARK" (0: Télescope parqué) /!\ Modif à faire aussi sur auxiliaire (+ fabriquer un dongle pour la maintenance)
- Ajout de barrières IR/US
- Coupure automatique de l'alimentation 19V

- Ajout de capteurs "portes fermées"
- Modification code (prise en compte des capteurs portes fermées)
- Accès réseau (Indi)
- Ajout d'un détecteur PIR

- NTP ou RTC/GPS
- Fermeture automatique de l'abri au lever du soleil
- Ouverture programmée de l'abri


TESTER
======
- Fiabilité réseau Ethernet

OPTIONS:
--------
- Ajout afficheur Oled

NOTES
=====
- Abri ni ouvert, ni fermé -> considéré comme ouvert pour les commandes
