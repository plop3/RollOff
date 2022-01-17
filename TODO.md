TODO
- Retour des infos en commande manuelle
- Nettoyage et refonte de la partie rolloffindi
- Gestion des boutons (clef + bouton vert)
  - Appui long: ouverture/fermeture des portes
      Si portes ouvertes: attente 3s puis fermeture des portes si bouton ON, sinon, ouverture de l'abri
- Gestion de l'afficheur Oled
- Gestion des leds (verte et bleue)
  - Verte: Abri ouvert
  - Bleue: mode manuel
- Commandes par port série (commandes Indi)
- Arrets d'urgence
- Gestion des sécurités:
  - Déplacement non prévu de l'abri ouvert
  - Le télescope bouge pendant le déplacement ou la fermeture des portes
- Gestion des éclairages
- Commandes par Réseau
- Park du télescope
  - Commandes réseau
  - Commande directe par pin Park
- Gestion du bouton rouge (ouvre/ferme porte panneau à flats)
- Gestion du mode manuel:
  - Bouton noir: 
      - Appui long: mode manuel/auto
      - Appui court: afficheur Oled on/off (5mn)
  - Bouton rouge: sélection de la fonction
  - Bouton vert: exécution de la fonction
- AJout du capteur de pluie
- Gestion arrêt (pas ARU) ?
- Commandes additionnelles:
  - Ouvre portes
  - Ferme portes
  - Ouvre porte 1
  - Ferme porte 1
  - Gestion des éclairages
  - MQTT (état abri, portes, park, éclairages)
  - Serveur Web (infos + commandes)

IndiDuino:
----------
- Ajout Lock
- Ajout Aux
- Lock: non ARU ?
NOTES
- Abri ouvert et fermé -> Erreur
- Abri ni ouvert, ni fermé -> considéré comme ouvert pour les commandes