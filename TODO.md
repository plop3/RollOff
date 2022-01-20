TODO
- [OK] Bouton vert: ouvre/ferme abri (portes appui long)
- [OK] Bouton rouge: Ouvre/ferme porte 2 (si porte 1 ouverte)
- Bouton noir+rouge: Ferme l'abri de façon inconditionnelle
- Bouton noir+vert: Déplace l'abri de façon inconditionnelle  
- Gestion AUTO/MANUEL (Indi/Boutons)                                                           

- Console de déboguage telnet
- Accès réseau (Indi)
OK (A tester)
- Vérifier qu'Indi ne puisse pas envoyer une commande pendant le déplacement
- Gestion de l'alimentation moteur
- Gestion stop abri (sans ARU)
    En cours de déplacement -> Arrête le moteur abri
    En cours de fermeture des portes -> bloque les portes
- Gestion ARU
- Surveillance
- Ajout détection Park
- Gestion des éclairages

- Vérifier le retour des infos en commande manuelle
- Vérifier qu'on ne puisse pas relancer une commande de déplacement tant que l'abri se déplace
- Gestion des éclairages
- Tests complets (à définir)
- Commandes par Réseau
- Park du télescope
  - Commandes réseau
  - Commande directe par pin Park
- Bouton noir: ouvre/ferme la porte 1 (pour réglages panneau à flats)
- AJout du capteur de pluie
- Gestion arrêt (pas ARU) ?
- Commandes additionnelles:
  - Gestion des éclairages
  - MQTT (état abri, portes, park, éclairages)
  - Serveur Web (infos + commandes)
- Gestion de l'afficheur Oled
- Gestion des leds (verte et bleue)
  - Verte: Abri ouvert
  - Bleue: mode manuel

- Gestion du mode manuel:
  - Bouton noir: 
      - Appui long: mode manuel/auto
      - Appui court: afficheur Oled on/off (5mn)
  - Bouton rouge: sélection de la fonction
  - Bouton vert: exécution de la fonction
    - Fonctions:
        - Ouvre portes
        - Ferme portes
        - Ouvre porte 1
        - Ferme porte 1
        - Ouvre porte 2
        - Déplace abri (validation si portes non ouvertes)
        - Réglage des éclairages
        - Gestion park O/N ?

IndiDuino:
----------
- Ajout Lock
- Ajout Aux
- Lock: non ARU ?
NOTES
- Abri ouvert et fermé -> Erreur
- Abri ni ouvert, ni fermé -> considéré comme ouvert pour les commandes
