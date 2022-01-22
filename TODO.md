TODO
====
- [OK] Bouton vert: ouvre/ferme abri (portes appui long)
- [OK] Bouton rouge: Ouvre/ferme porte 2 (si porte 1 ouverte)
- [OK] Bouton noir+rouge: Ferme l'abri de façon inconditionnelle
- [OK] Bouton noir+vert: Déplace l'abri de façon inconditionnelle  
- [OK] Gestion AUTO/MANUEL (Indi/Boutons)      
- Câbler le reset sur le shield

- Reset du module réseau au démarrage
- MQTT (état abri, portes, park, éclairages)
- Serveur Web (infos + commandes)

NOK

A TESTER
- AJout du capteur de pluie
- Gestion des éclairages
- Ferme abri: ajouter le park du télescope                                                     
- Park du télescope
  - Commandes réseau
  - Commande directe par pin Park
- Marche/arret des éclairages
- Alimentation télescope
- Commande lock
- Commande auxiliaire
- Commande stop
- Vérifier qu'Indi ne puisse pas envoyer une commande pendant le déplacement
- Gestion stop abri (sans ARU)
    En cours de déplacement -> Arrête le moteur abri
    En cours de fermeture des portes -> bloque les portes
- Gestion ARU
- Surveillance
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
