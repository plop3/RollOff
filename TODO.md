TODO
====
- [OK] Bouton vert: ouvre/ferme abri (portes appui long)
- [OK] Bouton rouge: Ouvre/ferme porte 2 (si porte 1 ouverte)
- [OK] Bouton noir+rouge: Ferme l'abri de façon inconditionnelle
- [OK] Bouton noir+vert: Déplace l'abri de façon inconditionnelle  
- [OK] Gestion AUTO/MANUEL (Indi/Boutons)                                                           
- Câbler le reset sur le shield
- Marche/arret des éclairages
- Reset du module réseau au démarrage
- Park du télescope
- Commandes réseau
- Commande directe par pin Park
- AJout du capteur de pluie
- Gestion des éclairages
- MQTT (état abri, portes, park, éclairages)
- Serveur Web (infos + commandes)
- Gestion des leds (verte et bleue)
- Verte: Télescope parqué
- Bleue: Déplacement en cours

OK (A tester)
- Vérifier qu'Indi ne puisse pas envoyer une commande pendant le déplacement
- Gestion de l'alimentation moteur
- Gestion stop abri (sans ARU)
    En cours de déplacement -> Arrête le moteur abri
    En cours de fermeture des portes -> bloque les portes
- Gestion ARU
- Surveillance
- Ajout détection Park
- Tests complets (à définir)
- Gestion arrêt (pas ARU) ?

IndiDuino:
----------
- Ajout Lock
- Ajout Aux

NOTES
=====
- Abri ouvert et fermé -> Erreur
- Abri ni ouvert, ni fermé -> considéré comme ouvert pour les commandes
