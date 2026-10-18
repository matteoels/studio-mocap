# Studio Mocap

Caméra → animation 3D en direct, dans le navigateur.

Site : https://matteoels.github.io/studio-mocap/

## Mode « Mon corps »

Anime en direct un modèle riggé (.glb, .gltf, .fbx, .vrm) avec ton corps : suivi des doigts et de la tête, deuxième webcam de côté optionnelle (calibration T-pose), enregistrement, coupe et export .glb.

## Mode « Objet tenu (cube) »

Tu tiens un cube de 10 cm, et le site suit sa position et sa rotation avec la webcam. L'enregistrement s'exporte en **texte pour GDevelop**, que l'extension **MocapPlayer** rejoue sur n'importe quel objet 3D ou sur la caméra. Le cube porte 6 marqueurs imprimés :

1. Imprime [`marqueurs/marqueurs_cube_10cm.pdf`](marqueurs/marqueurs_cube_10cm.pdf) **à 100 %** (pas « ajuster à la page ») : le carré noir doit mesurer 8,0 cm.
2. Découpe sur le trait gris et colle chaque carré sur sa face, la petite flèche ↑ vers la face indiquée :

   | Face | ID | Flèche ↑ vers |
   |---|---|---|
   | Avant | 4 | Dessus |
   | Arrière | 5 | Dessus |
   | Dessus | 2 | Arrière |
   | Dessous | 3 | Avant |
   | Droite | 0 | Dessus |
   | Gauche | 1 | Dessus |

   (droite et gauche quand tu regardes la face Avant, Dessus en haut)
3. Choisis « Objet tenu (cube) », démarre la caméra. Rien à calibrer : les marqueurs vus s'allument en vert.
4. Bouton rouge pour enregistrer, coupe si besoin, puis **Exporter pour GDevelop**.

Une seule face visible suffit pour la position et la rotation. Garde les doigts sur les arêtes et la marge blanche intacte autour de chaque carré noir : un marqueur touché par un doigt est ignoré (les autres prennent le relais). Si tu imprimes à une autre taille, indique le côté du carré noir et la taille du cube.

Le suivi tient avec une lumière chaude (lampe d'intérieur) et un peu de flou de mouvement : un marqueur flou est retrouvé grâce à la position prévue du cube. Les images où le cube n'est pas vu sont interpolées, et le lissage de la prise est centré dans le temps (pas de retard).

### Dans GDevelop

Importe [`gdevelop/MocapPlayer.json`](gdevelop/MocapPlayer.json) (Gestionnaire de projet → Extensions → Importer), colle le texte exporté dans une variable texte ou un objet Texte, puis :

- **Objet 3D** : comportement « Animation enregistrée », action *Jouer l'animation* (texte, boucle, vitesse, échelle en px/cm, déplacements, rotations, relatif à l'angle de départ, aller-retour). En relatif, le mouvement tourne avec l'objet : « loin de la webcam » = devant l'objet (angle 0 = vers +X), et les rotations se font autour de ses propres axes.
- **Caméra** : action *Jouer l'animation sur la caméra* (même réglages + calque).
- **Aller-retour** : à la fin, l'animation se rejoue à l'envers, donc l'objet ou la caméra revient exactement à son point de départ ; avec la boucle, ça se répète sans à-coup.

Format du texte : `SMOCAP1|30|<images>|x,y,z,qx,qy,qz,qw;...` — positions en cm et rotations (quaternion), relatives au début de la prise, dans le repère X à droite, Y en s'éloignant de la webcam, Z vers le haut.
