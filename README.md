# Studio Mocap

Caméra → animation 3D en direct, dans le navigateur.

Site : https://matteoels.github.io/studio-mocap/

## Mode « Mon corps »

Anime en direct un modèle riggé (.glb, .gltf, .fbx, .vrm) avec ton corps : suivi des doigts et de la tête, deuxième webcam de côté optionnelle (calibration T-pose), enregistrement, coupe et export .glb.

## Mode « Objet tenu (cube) »

Tu tiens un cube dont chaque face est peinte d'une couleur différente (un carton fait l'affaire). Le site suit sa position et sa rotation avec la webcam, et l'enregistrement s'exporte en **texte pour GDevelop**, que l'extension **MocapPlayer** rejoue sur n'importe quel objet 3D ou sur la caméra.

1. Choisis « Objet tenu (cube) », démarre la caméra.
2. **Calibrer les couleurs** : clique sur chaque face dans l'image de la webcam, dans l'ordre demandé (les couleurs sont gardées pour la prochaine fois).
3. Indique la taille du cube (cm). Si les distances semblent fausses, ajuste le champ de vision de la webcam.
4. Bouton rouge pour enregistrer, coupe si besoin, puis **Exporter pour GDevelop**.

Conseils :
- Des couleurs franches et bien différentes ; évite une couleur « entre deux » (orange avec rouge et jaune) et les couleurs du fond (blanc devant un mur blanc).
- Plante un crayon dans la face du dessous pour tenir le cube sans cacher les faces.
- Au début de la prise, montre deux ou trois faces : avec une seule face visible, le cube a quatre orientations possibles.

### Dans GDevelop

Importe [`gdevelop/MocapPlayer.json`](gdevelop/MocapPlayer.json) (Gestionnaire de projet → Extensions → Importer), colle le texte exporté dans une variable texte ou un objet Texte, puis :

- **Objet 3D** : comportement « Animation enregistrée », action *Jouer l'animation* (texte, boucle, vitesse, échelle en px/cm, déplacements, rotations, relatif à l'angle de départ).
- **Caméra** : action *Jouer l'animation sur la caméra* (même réglages + calque).

Format du texte : `SMOCAP1|30|<images>|x,y,z,qx,qy,qz,qw;...` — positions en cm et rotations (quaternion), relatives au début de la prise, dans le repère X à droite, Y en s'éloignant de la webcam, Z vers le haut.
