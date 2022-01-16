Pilotage de l'abri roulant pour télescope
Basé sur: https://github.com/wotalota/indi-rolloffino

Bouton à clef pour ouverture de l'abri:
    Appui long: Ouverture, fermeture des portes seules
    Appui court: Ouverture de l'abri, fermeture de l'abri
  
    Bouton vert à l'intérieur:
    Commandes identiques au bouton à clef.
    2 boutons arrêt d'urgence (1 intérieur, 1 extérieur)
    2 capteurs de position de l'abri (ouvert, fermé)
    1 capteur de position du télescope (parqué: true)
    2 capteurs d'ouverture des portes
  
TODO:
- Eclairage de l'abri (APA106)
- Pilotage Indi par réseau
- Pilotage par MQTT (Home Assistant)
- Pilotage par serveur Web
- Park du télescope avant fermeture
- Détection de pluie
- Détection d'intrusion par capteur PIR
- Alerte par SMS en cas de problème.  
- Arret de l'alimentation 12V
  
OPTIONS:
- Barrière(s) IR de sécurité
- Capteurs fermeture des portes
- Clavier codé
  
L'abri est une petite cabane sur rails.
- L'ouverture/fermeture des portes est assurée par 2 vérins électriques.
- L'ouverture/fermeture de l'abri est commandée par une motorisation de porte de garage.
- Ouverture/fermeture du dome par contacteur à clef, bouton.

MATERIEL

    Abri en bois roulant sur rails.
    2 vérins électriques pour l'ouverture/fermeture des portes.
    1 moteur de porte de garage pour le déplacement de l'abri.
    1 carte Arduino Mega
    1 carte 4 relais.
    1 alimentation 220V/12V
    1 convertisseur 12V/5V
