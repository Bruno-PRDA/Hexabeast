# Du modèle SolidWorks à Gazebo

La simulation lit une description URDF. L'exporteur gratuit **SolidWorks to
URDF** (sw2urdf, `github.com/ros/solidworks_urdf_exporter`) la produit
depuis l'assemblage : repères, axes, masses, inerties, limites, maillages.
Ce qu'il sort remplace `urdf/hexapod_body.urdf.xacro` ; le reste de la chaîne
(contrôleurs, capteurs, Gazebo) ne bouge pas **si les noms sont respectés**.

## Ce qui doit sortir de SolidWorks

| Élément | Convention attendue |
|---|---|
| Corps | lien `base_link`, repère au centre du corps, **X vers l'avant, Y vers la gauche, Z vers le haut** |
| Liens de patte | `FL_coxa_link`, `FL_femur_link`, `FL_tibia_link` — idem pour `ML`, `RL`, `FR`, `MR`, `RR` (F/M/R = avant/milieu/arrière, L/R = gauche/droite) |
| Articulations | `FL_coxa`, `FL_femur`, `FL_tibia`, … ; type *revolute* |
| Axe de la hanche (coxa) | Z du repère de l'articulation, positif = la patte tourne vers la gauche de sa direction de repos |
| Axes fémur et tibia | **−Y** du repère de l'articulation, positif = le segment se lève |
| Zéro des articulations | patte tendue : coxa dans sa direction de repos, fémur dans l'axe de la coxa, tibia dans l'axe du fémur |
| Repère de chaque articulation | origine sur l'**axe de rotation du servo** (pas au centre de la pièce), X pointant vers l'articulation suivante |
| Unités | mètres et kilogrammes — sw2urdf convertit depuis les mm, vérifier `<mass>` et les `<origin>` à l'export |

Les six directions de repos actuelles (angle depuis +X) : FL 45°, ML 90°,
RL 135°, FR −45°, MR −90°, RR −135°. Elles ne sont pas imposées : si le châssis
place les hanches ailleurs, on met à jour `LEGS` dans `robot_config.py` et
`tools/gen_urdf.py --check` valide que la démarche est toujours atteignable.

## Dans sw2urdf, concrètement

1. Créer dans l'assemblage un **point** et un **axe** de référence par
   articulation, sur l'axe de sortie du servo. L'exporteur s'en sert pour
   placer l'origine et l'axe du joint.
2. Regrouper les pièces en 19 liens : le corps (avec servos de hanche,
   électronique, batterie) et 3 liens par patte. Un servo appartient au lien
   qui le porte, pas à celui qu'il entraîne.
3. Affecter les matériaux (PLA, aluminium, masse des servos en pièces
   « pleines » avec densité ajustée) : ce sont eux qui donnent masses et
   inerties, et le verdict sur le couple en dépend directement.
4. Exporter. sw2urdf produit un paquet avec `urdf/` et `meshes/` (STL).

## Brancher l'export

1. Copier le contenu de `<robot>` du fichier exporté dans
   `ros2_ws/src/hexapod_description/urdf/hexapod_body.urdf.xacro`, et les
   maillages dans `hexapod_description/meshes/` (ajouter `meshes` à
   `install(DIRECTORY …)` dans `CMakeLists.txt`). Les chemins de maillage
   deviennent `package://hexapod_description/meshes/….STL`.
2. Garder des **collisions simples** : boîte pour le corps, cylindre pour le
   tibia et une sphère au bout du pied, comme dans le fichier généré. Un
   maillage détaillé en collision rend la physique lente et instable sans rien
   apporter. La sphère du pied doit rester la première collision du tibia :
   c'est elle que le capteur de contact référence.
3. Reporter dans `robot_config.py` ce que la CAO a fixé : longueurs d'axe à
   axe, positions des hanches, masses. Lancer `python tools/gen_urdf.py --check`
   pour régénérer contrôleurs et pont, et vérifier la cinématique.
4. `ros2 launch hexapod_description display.launch.py` : bouger chaque
   curseur, vérifier que fémur et tibia se **lèvent** pour un angle positif et
   que la coxa tourne vers la gauche. Si un axe est inversé, corriger le signe
   de `<axis>` plutôt que le code.

## Ce que SolidWorks ne remplace pas

SolidWorks Motion sait faire de la cinématique, pas la boucle de commande des
servos ni la démarche : c'est le rôle de Gazebo et du nœud `hexapod_gait`.
Les deux sont complémentaires — SolidWorks dit *ce qu'est* le robot, Gazebo
dit *ce qu'il fait*.
