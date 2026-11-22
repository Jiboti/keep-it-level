<h1 align="center">🎯 Keep It Level</h1>
<h4 align="center">Plateforme stabilisée compacte pour antenne GNSS anti-brouillage de drone · European Defense Tech Hackathon — Paris 2026</h4>

<p align="center">
  <b>🏅 Mention honorable</b> — Défi 02 « Keep It Level », proposé par <b>Fire Point</b><br>
  Équipe <b>Proteus Group</b> · 20–22 novembre 2026
</p>

<p align="center">
  <img src="media/prototype.jpg" alt="Prototype de la plateforme stabilisée" width="600"/>
  <br><sub><em>Le prototype en fonctionnement</em></sub>
</p>

> ℹ️ **Ce dépôt couvre la mécanique et le firmware de la plateforme.** Le modèle de commande et la simulation sur logs de vol réels ont été menés en parallèle par un autre membre de l'équipe et sont conservés dans un dépôt séparé.

---

## 🎯 Le défi

Quand un drone manœuvre, sa cellule s'incline en roulis et en tangage, et tout ce qui est fixé dessus s'incline avec elle. Pour une antenne GNSS anti-brouillage à diagramme contrôlé (**CRPA**), c'est un problème : elle doit rester alignée sur la verticale locale pour bien recevoir le signal, précisément pendant les phases de vol où le drone manœuvre le plus. Les plateformes gyrostabilisées existantes sont trop grosses, trop lourdes et trop chères pour de petits drones.

**Objectif :** concevoir une plateforme stabilisée compacte et peu coûteuse qui garde l'antenne à l'horizontale, à partir des données d'attitude du contrôleur de vol **Cube Orange** (ArduPilot).

| Exigence du cahier des charges | Valeur |
|---|---|
| Inclinaison de la cellule (roulis et tangage) | jusqu'à 45° |
| Vitesse angulaire, turbulence comprise | 1,5 à 25 °/s |
| Fréquence des perturbations | 0,1 à 1,2 Hz |
| Erreur de mise à niveau | ≤ 2° |
| Masse de l'antenne | 2 kg (minimum), 4 kg (option) |
| Dimensions de l'antenne | 400 × 400 mm, la plateforme ne doit pas dépasser |

**Livrables attendus :** concept (schémas fonctionnel et électronique, modèle 3D, liste de composants justifiée), modèle mathématique de la commande (code source), simulation sur données réelles du Cube Orange.

## 💡 Notre solution

Un **cardan 2 axes** (roulis à l'extérieur, tangage à l'intérieur), avec sur chaque axe **un servomoteur d'un côté et un pivot de l'autre**, piloté en **compensation directe** de l'attitude du drone.

```
 Attitude du drone (IMU / Cube Orange)
            │
            ▼
 Filtre complémentaire (gyro 98 % + accéléromètre 2 %)
            │  roulis φ, tangage θ
            ▼
 PID de suivi de consigne (par axe)  ──►  consignes  q_roulis = −φ ,  q_tangage = −θ
            │
            ▼
 Servomoteurs numériques (PID de position interne)
            │
            ▼
 Cardan 2 axes  ──►  antenne à l'horizontale
```

**Pourquoi le roulis à l'extérieur et le tangage à l'intérieur ?** Avec la convention d'angles d'ArduPilot (lacet, tangage, roulis), cet ordre découple exactement les deux axes : il suffit d'appliquer à chaque servo l'opposé de l'angle mesuré, sans approximation des petits angles.

### Choix du principe de stabilisation

| Principe | Verdict |
|---|---|
| Cardan 2 axes à moteurs brushless (type nacelle caméra) | Meilleure solution industrielle, trop longue à mettre au point en 48 h → **version 2** |
| **Cardan 2 axes à servomoteurs** | **Retenu** : asservissement de position intégré, rapide à concevoir, simuler et fabriquer |
| Mécanisme parallèle sous l'antenne | Cinématique couplée, courses trop longues pour 45° |
| Pendule passif amorti | Éliminé : en virage coordonné, il s'aligne sur la gravité apparente et penche avec le drone |

## 🔧 Mécanique

Pièces conçues sous **SolidWorks** et imprimées en 3D (Bambu Lab), pensées pour l'impression : piliers en prismes droits sans porte-à-faux, tourillons en goutte d'eau imprimables sans support, congés à toutes les jonctions.

| Pièce | Rôle | Encombrement |
|---|---|---|
| Plateforme de transition | Cadre qui tourne autour de l'axe X (roulis) ; porte le servo de tangage, le pivot du plateau et un caisson de contrepoids | 98,5 × 151 × 18,5 mm |
| Plateau intérieur | Support de l'antenne, tourne autour de l'axe Y (tangage) | 70 × 70 mm |
| Pilier servo | Reçoit le servo de roulis, fixé par ses pattes | pied 60 × 60, axe à 70 mm du sol |
| Pilier pivot | Tourillon Ø6 en goutte d'eau, dans l'axe du servo | pied 60 × 60, axe à 70 mm du sol |

- **Actionneurs du prototype :** 2 × servos numériques EMAX ES09MD (pignons métal), palonnier double bras sur l'axe X, simple bras sur l'axe Y.
- **Pivots :** tourillons Ø6 dans des alésages Ø6,4.
- **Équilibrage :** caisson de contrepoids (environ 12,6 g) pour ramener le centre de gravité de la partie mobile sur l'axe de roulis.

### Vérifications effectuées sur la CAO

| Vérification | Résultat |
|---|---|
| Collisions sur tout le débattement | aucune sur ±50° en roulis × ±50° en tangage (spécification : 45°) |
| Couple demandé au servo (charge de 150 g sur le plateau) | 0,028 N·m, pour environ 0,075 N·m tenus en continu par le ES09MD |
| Emboîtement servos, palonniers et tourillons | aucune interférence |
| Imprimabilité | piliers sans support ; maillages STL étanches |

## ⚡ Électronique et commande

- **Mesure d'attitude :** IMU (accéléromètre + gyroscope) placée sur la base, dans le rôle du Cube Orange qui fournit l'attitude du drone.
- **Filtre complémentaire :** 98 % gyroscope (insensible aux accélérations des manœuvres) et 2 % accéléromètre (corrige la dérive). Pendant les mouvements rapides, c'est le gyroscope qui donne l'angle, ce qui supprime les vibrations dues aux accélérations.
- **PID par axe :** fait suivre au servo la consigne de compensation sans dépassement, avec limite de vitesse, butées logicielles et zone morte de 0,3°.
- **Commande des servos** en microsecondes (résolution fine), boucle à 100 Hz.

Code : [`firmware/stabilisation_EDTH_PID.cpp`](firmware/stabilisation_EDTH_PID.cpp)

### Réglages principaux

| Paramètre | Rôle | Valeur de départ |
|---|---|---|
| `Factor` | Poids du gyroscope dans le filtre | 0,98 |
| `KP`, `KI`, `KD` | PID de suivi de consigne | 8 ; 0,5 ; 0,15 |
| `GAIN` | Amplitude de la compensation | 1,0 (à baisser si le plateau surcompense) |
| `US_PAR_DEGRE` | Étalonnage du servo (µs par degré) | à mesurer au rapporteur |

## 📈 Validation par la simulation

En parallèle de la conception mécanique, l'équipe a simulé la chaîne complète sur des **logs d'attitude réels** (vols ArduPlane) et sur des vols synthétiques, avec le critère d'erreur ≤ 2°.

| Scénario | Erreur max | Sans prédicteur |
|---|:---:|:---:|
| Vol synthétique nominal (125 s) | 1,20° ✅ | 2,73° ✗ |
| Vol synthétique de stress, 45° sur les deux axes (50 s) | 1,27° ✅ | 3,51° ✗ |
| Vol réel 1 (ArduPlane, fenêtre 120 s) | 1,62° ✅ | 4,25° ✗ |
| Vol réel 2 (ArduPlane, fenêtre 120 s) | 1,88° ✅ | 8,61° ✗ |

Trois enseignements notables :

- **La prédiction est indispensable.** Sans compensation du retard de liaison (40 ms), le critère de 2° est raté dans tous les scénarios.
- **La course mécanique doit atteindre ±55° par axe.** À ±50°, le vol réel 1 (roulis jusqu'à 54,4°) donne 4,29° d'erreur ; à ±45°, 9,08°.
- **Le centrage de la masse est critique.** Avec 4 kg et un centre de gravité décalé de 10 mm, le balourd seul dépasse le couple continu estimé : il faut centrer à ±3 mm.

## 🚀 Du prototype à l'antenne réelle

Le prototype est une **maquette à échelle réduite** (plateau de 70 × 70 mm). Le dimensionnement pour l'antenne réelle de 400 × 400 mm a été fait en parallèle :

| Grandeur | Antenne de 2 kg | Antenne de 4 kg |
|---|---|---|
| Inertie autour d'un axe | ≈ 0,04 kg·m² | ≈ 0,08 kg·m² |
| Couple continu nécessaire | ≈ 0,8 N·m | ≈ 1,5 N·m |

- **Bande passante :** au moins 2 Hz en boucle fermée pour suivre 25 °/s avec moins de 2° d'erreur ; viser 5 Hz.
- **Retard total :** moins de 40 ms entre la mesure et la correction.
- **Actionneurs recommandés :** servos intelligents de 4 à 10 N·m (type Dynamixel XM430 / XM540) pour un prototype à l'échelle 1, puis moteurs brushless en entraînement direct pour la version industrielle.
- **Point clé :** équilibrer l'antenne sur ses axes divise le couple nécessaire par deux.

### Suite du projet

- [ ] Reprendre la CAO pour une course de ±55° par axe (résultat de la simulation)
- [ ] Lecture de l'attitude du Cube Orange en MAVLink (message `ATTITUDE`) à la place de l'IMU embarquée
- [ ] Prototype à l'échelle 1 avec antenne de 2 kg et actionneurs dimensionnés
- [ ] Ajout d'une IMU sur la plateforme pour mesurer l'erreur réelle et corriger le calage

## 📁 Contenu du dépôt

```
├── cad/          # Pièces SolidWorks, STEP et STL prêts à imprimer
├── firmware/     # Code de stabilisation (filtre complémentaire + PID)
├── docs/         # Plans cotés, deck de présentation
└── media/        # Photos et vidéos du prototype
```

### Impression

- Matière : PETG ou PLA+
- Couches de 0,2 mm, 4 parois, remplissage 40 % gyroïde
- Piliers debout sans support ; plateforme et plateau à plat avec supports arborescents posés sur le plateau uniquement
- Visserie : vis autotaraudeuses M2 × 8 pour les pattes des servos

## 👥 L'équipe — Proteus Group

| Membre | Rôle |
|---|---|
| Jaime SOUSA | Conception mécanique (CAO) et firmware de stabilisation |
| Tristan GAVEAU | Modèle de commande, simulation et validation sur logs réels |
| *(à compléter)* | *(à compléter)* |
| *(à compléter)* | *(à compléter)* |
| *(à compléter)* | *(à compléter)* |

**Ma contribution :** la conception mécanique complète du cardan 2 axes sous **SolidWorks** — plateforme de transition, plateau intérieur, piliers servo et pivot — pensée pour l'impression 3D (pas de porte-à-faux, tourillons en goutte d'eau) et vérifiée en collision sur tout le débattement. J'ai également développé le **firmware de stabilisation** : filtre complémentaire gyroscope/accéléromètre et PID de suivi de consigne par axe, avec limite de vitesse, butées logicielles et zone morte. Enfin, j'ai mené le dimensionnement de la transposition vers l'antenne réelle de 400 × 400 mm (inerties, couples, choix d'actionneurs).

## 🙏 Remerciements

Merci à **Fire Point** pour ce défi très concret, aux mentors, et aux organisateurs et partenaires de l'**European Defense Tech Hackathon — Paris 2026**.

---

<p align="center"><a href="https://github.com/Jiboti">⬅️ Retour au profil</a></p>
