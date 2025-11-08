# Description des Options du Programme Orthophoto & Mesh Tool 2.0

Ce document décrit en détail toutes les options et paramètres disponibles dans l'application **Orthophoto & Mesh Tool 2.0**.

## Table des Matières
1. [Menu Fichier](#menu-fichier)
2. [Filtres PDAL](#filtres-pdal)
3. [Options de Mesh](#options-de-mesh)
4. [Modes d'Orthophoto](#modes-dorthophoto)
   - [Orthophoto en Élévation](#orthophoto-en-élévation)
   - [Orthophoto en Plan](#orthophoto-en-plan)
   - [Coins XYZ (Ancien)](#coins-xyz-ancien)
   - [Mode Batch](#mode-batch)
5. [Flux de Travail Typique](#flux-de-travail-typique)
6. [Traitement par Batch](#traitement-par-batch)

---

## Menu Fichier

Le menu Fichier permet de gérer les fichiers de travail pour sauvegarder et restaurer vos configurations d'orthophotos.

### Nouveau

- **Raccourci**: Menu Fichier → Nouveau
- **Description**: Crée un nouveau fichier de travail vierge
- **Effet**: Efface toutes les configurations de batch actuelles après confirmation

### Ouvrir

- **Raccourci**: Menu Fichier → Ouvrir...
- **Description**: Ouvre un fichier de travail existant (.orth)
- **Effet**: 
  - Charge le chemin du fichier de nuage de points
  - Charge toutes les configurations d'orthophotos sauvegardées
  - Restaure les paramètres de chaque configuration dans le batch

### Enregistrer

- **Raccourci**: Menu Fichier → Enregistrer
- **Description**: Enregistre le fichier de travail actuel
- **Effet**: 
  - Si le fichier existe déjà : écrase le fichier
  - Sinon : affiche la boîte de dialogue "Enregistrer sous"

### Enregistrer sous

- **Raccourci**: Menu Fichier → Enregistrer sous...
- **Description**: Enregistre le fichier de travail avec un nouveau nom
- **Format**: Fichiers .orth (format JSON)
- **Contenu sauvegardé**:
  - Chemin du fichier de nuage de points source
  - Toutes les configurations d'orthophotos du batch
  - Paramètres complets pour chaque mode (Élévation/Plan/XYZ)

### Quitter

- **Raccourci**: Menu Fichier → Quitter
- **Description**: Ferme l'application
- **Note**: Pensez à sauvegarder votre travail avant de quitter

---

## Filtres PDAL

Ces filtres permettent de nettoyer et optimiser le nuage de points avant traitement.

### Filtrer les outliers

- **Type**: Case à cocher
- **État par défaut**: Activé
- **Description**: Active la suppression automatique des points aberrants (outliers) du nuage de points.

### Multiplicateur d'outliers

- **Type**: Nombre décimal (double)
- **Plage**: 0.5 à 10.0
- **Valeur par défaut**: 2.0
- **Description**: Contrôle la sensibilité du filtre d'outliers. Une valeur plus élevée est plus permissive, une valeur plus faible supprime plus de points.

### Décimation du nuage

- **Type**: Case à cocher
- **État par défaut**: Désactivé
- **Description**: Active la réduction du nombre de points en conservant 1 point sur N.

### Pas de décimation

- **Type**: Nombre entier
- **Plage**: 2 à 100
- **Valeur par défaut**: 10
- **Description**: Conserve 1 point tous les N points. Par exemple, avec un pas de 10, seul 1 point sur 10 est conservé.

---

## Options de Mesh

Ces options contrôlent la génération du maillage 3D (mesh) à partir du nuage de points.

### Profondeur Poisson (Poisson Depth)

- **Type**: Nombre décimal (double)
- **Plage de valeurs**: 1 à 20
- **Valeur par défaut**: 8
- **Description**: 
  
  La profondeur Poisson contrôle la résolution du mesh généré par l'algorithme de reconstruction de surface de Poisson. C'est le paramètre le plus important pour la qualité du mesh.

  - **Valeurs faibles (1-5)**: 
    - Génère un mesh grossier avec moins de détails
    - Traitement plus rapide
    - Fichier de sortie plus petit
    - Recommandé pour des prévisualisations rapides
    
  - **Valeurs moyennes (6-10)**: 
    - Bon équilibre entre détails et performance
    - Recommandé pour la plupart des applications
    - La valeur par défaut de 8 convient à la majorité des cas d'usage
    
  - **Valeurs élevées (11-20)**: 
    - Génère un mesh très détaillé
    - Traitement beaucoup plus lent
    - Fichier de sortie volumineux
    - Recommandé uniquement pour des modèles finaux haute résolution
    - Nécessite plus de mémoire RAM

- **Impact technique**: 
  - Chaque niveau de profondeur double approximativement le nombre de polygones du mesh
  - La profondeur N génère environ 4^N cellules dans l'octree utilisé pour la reconstruction

### Colorier le mesh

- **Type**: Case à cocher (checkbox)
- **État par défaut**: Activé (coché)
- **Description**: 
  
  Cette option détermine si les couleurs du nuage de points original doivent être appliquées au mesh généré.

  - **Activé** (coché):
    - Le mesh généré conservera les informations de couleur du nuage de points source
    - Résultat visuellement plus réaliste
    - Fichier de sortie légèrement plus volumineux (données de couleur RGB pour chaque vertex)
    - Recommandé pour la génération d'orthophotos
    
  - **Désactivé** (non coché):
    - Le mesh sera généré sans information de couleur
    - Mesh monochrome (généralement gris ou blanc dans la visualisation)
    - Fichier de sortie plus petit
    - Utile pour l'analyse géométrique pure ou les tests de performance

- **Note d'implémentation**: 
  - Cette fonctionnalité est maintenant implémentée dans la version 2.0
  - Les couleurs sont transférées du nuage de points au mesh via recherche des plus proches voisins

---

## Modes d'Orthophoto

L'application propose trois modes de création d'orthophotos, chacun adapté à un cas d'usage spécifique.

### Orthophoto en Élévation

Ce mode permet de créer des orthophotos alignées sur un axe défini par deux points XY, idéal pour les façades de bâtiments ou les coupes verticales.

#### Point 1 - X et Y

- **Type**: Nombres décimaux (double)
- **Plage**: -1,000,000 à 1,000,000
- **Description**: Coordonnées du premier point définissant l'axe d'élévation

#### Point 2 - X et Y

- **Type**: Nombres décimaux (double)
- **Plage**: -1,000,000 à 1,000,000
- **Description**: Coordonnées du deuxième point définissant l'axe d'élévation
- **Note**: La distance entre Point 1 et Point 2 détermine la largeur de l'orthophoto

#### Z de départ

- **Type**: Nombre décimal (double)
- **Plage**: -1,000,000 à 1,000,000
- **Description**: Altitude de départ (base) de la boîte d'extraction
- **Unités**: Selon le système de coordonnées du nuage de points

#### Hauteur

- **Type**: Nombre décimal (double)
- **Plage**: 0.1 à 10,000
- **Valeur par défaut**: 10.0
- **Description**: Hauteur verticale de la boîte d'extraction (dimension en Z)
- **Unités**: Mètres (généralement)

#### Profondeur

- **Type**: Nombre décimal (double)
- **Plage**: 0.1 à 10,000
- **Valeur par défaut**: 5.0
- **Description**: Profondeur de la boîte perpendiculaire à l'axe défini
- **Unités**: Mètres (généralement)

#### Sens profondeur

- **Type**: Liste déroulante
- **Options**: "Avant" ou "Arrière"
- **Valeur par défaut**: Avant
- **Description**: 
  - **Avant**: La profondeur s'étend perpendiculairement vers l'avant de l'axe
  - **Arrière**: La profondeur s'étend perpendiculairement vers l'arrière de l'axe

#### Résolution

- **Type**: Nombre décimal (double)
- **Plage**: 1 à 1,000
- **Valeur par défaut**: 100.0
- **Unités**: Pixels par mètre
- **Description**: Contrôle la résolution de l'image exportée. 100 px/m = 1 cm par pixel

### Orthophoto en Plan

Ce mode permet de créer des orthophotos en vue de dessus (plan), avec une coupe horizontale à une altitude définie.

#### Coin bas - X et Y

- **Type**: Nombres décimaux (double)
- **Plage**: -1,000,000 à 1,000,000
- **Description**: Coordonnées du coin inférieur gauche du cadre de l'orthophoto

#### Hauteur de coupe Z

- **Type**: Nombre décimal (double)
- **Plage**: -1,000,000 à 1,000,000
- **Description**: Altitude à laquelle la coupe horizontale est effectuée
- **Unités**: Selon le système de coordonnées du nuage de points

#### Largeur cadre

- **Type**: Nombre décimal (double)
- **Plage**: 0.1 à 10,000
- **Valeur par défaut**: 10.0
- **Description**: Largeur du cadre (dimension en X)
- **Unités**: Mètres (généralement)

#### Hauteur cadre

- **Type**: Nombre décimal (double)
- **Plage**: 0.1 à 10,000
- **Valeur par défaut**: 10.0
- **Description**: Hauteur du cadre (dimension en Y)
- **Unités**: Mètres (généralement)

#### Profondeur

- **Type**: Nombre décimal (double)
- **Plage**: 0.1 à 10,000
- **Valeur par défaut**: 5.0
- **Description**: Profondeur verticale de capture autour de la hauteur de coupe
- **Unités**: Mètres (généralement)

#### Sens profondeur

- **Type**: Liste déroulante
- **Options**: "Vers le haut" ou "Vers le bas"
- **Valeur par défaut**: Vers le haut
- **Description**: 
  - **Vers le haut**: La profondeur s'étend vers le haut à partir de la hauteur de coupe
  - **Vers le bas**: La profondeur s'étend vers le bas à partir de la hauteur de coupe

#### Résolution

- **Type**: Nombre décimal (double)
- **Plage**: 1 à 1,000
- **Valeur par défaut**: 100.0
- **Unités**: Pixels par mètre
- **Description**: Contrôle la résolution de l'image exportée. 100 px/m = 1 cm par pixel

### Coins XYZ (Ancien)

Ce mode est conservé pour compatibilité avec les versions précédentes. Il définit une simple boîte englobante.

## Options de l'Orthophoto (suite)

Ces options définissent la zone d'intérêt et les limites spatiales pour l'export de l'orthophoto.

### Coins de l'orthophoto (XYZ)

L'orthophoto est définie par un parallélépipède rectangle (boîte englobante) dans l'espace 3D, spécifié par ses coins minimum et maximum.

#### Coordonnées Minimum

- **X min**: 
  - **Type**: Nombre décimal (double)
  - **Plage**: -1,000,000 à 1,000,000
  - **Description**: Coordonnée X minimale de la zone d'intérêt
  - **Unités**: Selon le système de coordonnées du nuage de points source (généralement mètres)

- **Y min**: 
  - **Type**: Nombre décimal (double)
  - **Plage**: -1,000,000 à 1,000,000
  - **Description**: Coordonnée Y minimale de la zone d'intérêt
  - **Unités**: Selon le système de coordonnées du nuage de points source (généralement mètres)

- **Z min**: 
  - **Type**: Nombre décimal (double)
  - **Plage**: -1,000,000 à 1,000,000
  - **Description**: Coordonnée Z minimale (altitude minimale) de la zone d'intérêt
  - **Unités**: Selon le système de coordonnées du nuage de points source (généralement mètres)

#### Coordonnées Maximum

- **X max**: 
  - **Type**: Nombre décimal (double)
  - **Plage**: -1,000,000 à 1,000,000
  - **Description**: Coordonnée X maximale de la zone d'intérêt
  - **Unités**: Selon le système de coordonnées du nuage de points source (généralement mètres)

- **Y max**: 
  - **Type**: Nombre décimal (double)
  - **Plage**: -1,000,000 à 1,000,000
  - **Description**: Coordonnée Y maximale de la zone d'intérêt
  - **Unités**: Selon le système de coordonnées du nuage de points source (généralement mètres)

- **Z max**: 
  - **Type**: Nombre décimal (double)
  - **Plage**: -1,000,000 à 1,000,000
  - **Description**: Coordonnée Z maximale (altitude maximale) de la zone d'intérêt
  - **Unités**: Selon le système de coordonnées du nuage de points source (généralement mètres)

### Mise à jour automatique des coins

Lorsque vous effectuez un **pré-traitement PDAL** sur un nuage de points, l'application calcule automatiquement les valeurs min/max pour X, Y et Z en fonction de l'étendue réelle du nuage de points. Ces valeurs sont automatiquement remplies dans les champs correspondants.

Vous pouvez ensuite ajuster manuellement ces valeurs pour :
- Réduire la zone d'intérêt à une région spécifique
- Exclure des zones périphériques non pertinentes
- Définir une altitude minimale/maximale pour filtrer certains éléments

### Utilisation des coins lors de l'export

Les coordonnées définies sont utilisées lors de l'export de l'orthophoto pour :
1. Déterminer la région du mesh à projeter
2. Calculer la résolution de l'image de sortie
3. Définir le système de référence spatial de l'orthophoto exportée

---

### Mode Batch

Le mode Batch permet de configurer et traiter plusieurs orthophotos en une seule opération. C'est idéal pour :
- Générer plusieurs vues d'un même bâtiment (façades multiples)
- Créer des orthophotos à différentes résolutions
- Automatiser la production d'orthophotos pour documentation

#### Liste des configurations

- **Type**: Liste déroulante avec sélection
- **Description**: Affiche toutes les configurations d'orthophotos ajoutées au batch
- **Format d'affichage**: `[Nom] [Mode] -> [Chemin de sortie]`
- **Exemple**: `Façade Nord [Élévation] -> /output/facade_nord.png`

#### Ajouter au batch

- **Type**: Bouton
- **Fonction**: Capture la configuration actuelle de l'onglet sélectionné et l'ajoute au batch
- **Processus**:
  1. Demande un nom pour la configuration
  2. Demande le chemin du fichier de sortie
  3. Capture tous les paramètres de l'onglet actif (Élévation/Plan/XYZ)
  4. Ajoute la configuration à la liste du batch

#### Retirer du batch

- **Type**: Bouton
- **Fonction**: Supprime la configuration sélectionnée de la liste du batch
- **Condition**: Une configuration doit être sélectionnée dans la liste

#### Vider le batch

- **Type**: Bouton
- **Fonction**: Supprime toutes les configurations du batch après confirmation
- **Usage**: Utile pour recommencer avec un batch vide

#### Traiter le batch

- **Type**: Bouton (mise en évidence)
- **Fonction**: Génère toutes les orthophotos configurées dans le batch
- **Processus**:
  1. Parcourt chaque configuration dans l'ordre
  2. Applique les paramètres de la configuration
  3. Exporte l'orthophoto vers le fichier spécifié
  4. Affiche la progression globale
  5. Présente un résumé (succès/échecs)

#### Avantages du mode Batch

- ✅ **Gain de temps**: Configurez une fois, générez plusieurs fois
- ✅ **Reproductibilité**: Sauvegardez vos configurations dans un fichier .orth
- ✅ **Automatisation**: Traitement automatique sans intervention manuelle
- ✅ **Documentation**: Les fichiers .orth documentent vos paramètres
- ✅ **Partage**: Partagez vos configurations avec d'autres utilisateurs

---

## Flux de Travail Typique

Voici comment utiliser ces options dans un flux de travail standard :

### Étape 1 : Sélection du fichier
1. Cliquez sur **"Sélectionner nuage de points"**
2. Choisissez un fichier .e57, .las ou .laz

### Étape 2 : Pré-traitement
1. Cliquez sur **"Pré-traitement PDAL"**
2. Les coins de l'orthophoto (X/Y/Z min/max) sont automatiquement calculés
3. Vérifiez et ajustez ces valeurs si nécessaire

### Étape 3 : Configuration du mesh
1. Ajustez la **Profondeur Poisson** selon vos besoins :
   - 6-8 pour un équilibre performance/qualité
   - 10-12 pour plus de détails
   - 3-5 pour des tests rapides
2. Cochez/décochez **"Colorier le mesh"** selon vos besoins

### Étape 4 : Génération du mesh
1. Cliquez sur **"Générer Mesh Poisson"**
2. Attendez la fin du traitement (peut prendre plusieurs minutes pour des profondeurs élevées)

### Étape 5 : Ajustement final des coins (optionnel)
1. Ajustez manuellement les coins de l'orthophoto si vous voulez :
   - Zoomer sur une zone spécifique
   - Exclure des bordures
   - Limiter la plage d'altitude

### Étape 6 : Export de l'orthophoto
1. Cliquez sur **"Exporter Orthophoto"**
2. Choisissez le format (PNG, JPEG, TIFF)
3. Spécifiez le nom et l'emplacement du fichier

### Étape 7 : Visualisation (optionnel)
1. Cliquez sur **"Visualiser"** à tout moment pour voir :
   - Le nuage de points (après pré-traitement)
   - Le mesh généré (après génération du mesh)

---

## Traitement par Batch

Le mode Batch offre un flux de travail optimisé pour générer plusieurs orthophotos à partir du même nuage de points.

### Flux de Travail en Mode Batch

#### Étape 1 : Préparation (identique au flux standard)
1. Sélectionnez votre fichier de nuage de points
2. Effectuez le pré-traitement PDAL
3. Générez le mesh Poisson

#### Étape 2 : Configuration des orthophotos
1. Allez dans l'onglet **Élévation**, **Plan** ou **XYZ** selon vos besoins
2. Configurez les paramètres pour la première orthophoto
3. Cliquez sur **"Ajouter au batch"**
4. Donnez un nom à la configuration (ex: "Façade Nord")
5. Spécifiez le chemin de sortie (ex: `/output/facade_nord.png`)
6. Répétez pour chaque orthophoto désirée

#### Étape 3 : Gestion du batch
- Allez dans l'onglet **Batch** pour voir toutes vos configurations
- Vérifiez la liste des configurations
- Supprimez ou modifiez si nécessaire
- **Important**: Les configurations sont exécutées dans l'ordre d'ajout

#### Étape 4 : Sauvegarde du fichier de travail (optionnel mais recommandé)
1. Menu **Fichier** → **Enregistrer sous...**
2. Choisissez un nom (ex: `projet_batiment_nord.orth`)
3. Le fichier sauvegarde :
   - Le chemin du nuage de points
   - Toutes les configurations du batch
   - Tous les paramètres de chaque orthophoto

#### Étape 5 : Traitement
1. Dans l'onglet **Batch**, cliquez sur **"Traiter le batch"**
2. L'application traite automatiquement chaque configuration
3. La barre de progression montre l'avancement global
4. Un résumé final indique le nombre de succès/échecs

#### Étape 6 : Réutilisation
1. Pour reprendre un projet : Menu **Fichier** → **Ouvrir...**
2. Sélectionnez votre fichier `.orth`
3. Toutes les configurations sont restaurées
4. Vous pouvez modifier, ajouter ou supprimer des configurations
5. Relancez le traitement si nécessaire

### Exemple d'Utilisation : Façades d'un Bâtiment

**Objectif** : Générer les 4 façades d'un bâtiment

**Configuration** :
1. **Façade Nord** : Élévation, axe Y=0 à Y=10, Z=0 à Z=15
2. **Façade Sud** : Élévation, axe Y=50 à Y=60, Z=0 à Z=15
3. **Façade Est** : Élévation, axe X=0 à X=10, Z=0 à Z=15
4. **Façade Ouest** : Élévation, axe X=50 à X=60, Z=0 à Z=15

**Processus** :
1. Chargez le nuage de points du bâtiment
2. Pré-traitez et générez le mesh
3. Ajoutez les 4 configurations au batch
4. Sauvegardez le fichier de travail `batiment_facades.orth`
5. Cliquez sur "Traiter le batch"
6. Récupérez vos 4 orthophotos dans les fichiers spécifiés

**Avantages** :
- Génération automatique des 4 façades
- Sauvegarde du projet pour futures modifications
- Reproductibilité exacte du processus
- Possibilité de modifier une seule façade et re-générer

### Cas d'Usage Avancés

#### Multi-résolution
Créez plusieurs orthophotos de la même zone avec différentes résolutions :
- Configuration 1 : 50 px/m (aperçu)
- Configuration 2 : 100 px/m (standard)
- Configuration 3 : 200 px/m (haute résolution)

#### Coupes Multiples
Pour un plan en toiture :
- Configuration 1 : Coupe à Z=10m (niveau 1)
- Configuration 2 : Coupe à Z=13m (niveau 2)
- Configuration 3 : Coupe à Z=16m (niveau 3)

#### Multi-format
Générez la même orthophoto dans différents formats :
- Configuration 1 : Sortie PNG (archivage)
- Configuration 2 : Sortie JPEG (partage)
- Configuration 3 : Sortie TIFF (géoréférencement)

---

## Conseils et Bonnes Pratiques

### Pour la Profondeur Poisson :
- **Commencez toujours avec la valeur par défaut (8)** et ajustez si nécessaire
- **Augmentez progressivement** (8 → 9 → 10) plutôt que de sauter directement à 15
- **Surveillez l'utilisation mémoire** pour les valeurs > 12
- **Pour des nuages de points denses** (> 10 millions de points), une profondeur de 9-11 est généralement suffisante

### Pour le Coloriage du Mesh :
- **Laissez activé** si vous générez une orthophoto finale
- **Désactivez** uniquement pour les tests de géométrie ou pour économiser de la mémoire

### Pour les Coins de l'Orthophoto :
- **Ne modifiez pas** les valeurs auto-calculées sauf si nécessaire
- **Vérifiez toujours** que X_max > X_min, Y_max > Y_min, Z_max > Z_min
- **Utilisez des marges** légèrement plus grandes que nécessaire pour éviter de couper des éléments importants
- **Pour des zones urbaines**, une précision au mètre près est généralement suffisante
- **Pour des relevés de précision**, vous pouvez avoir besoin d'une précision au centimètre

### Performances et Temps de Traitement :
- **Profondeur 6-8** : Quelques secondes à quelques minutes
- **Profondeur 9-11** : Quelques minutes à 15 minutes
- **Profondeur 12-15** : 15 minutes à plusieurs heures
- **Profondeur 16+** : Plusieurs heures, nécessite > 16 GB RAM

---

## Formats de Fichiers Supportés

### Entrée (Nuage de points) :
- **.e57** : Format E57 ASTM (LiDAR)
- **.las** : Format LAS (LiDAR)
- **.laz** : Format LAZ compressé (LiDAR)

### Sortie (Orthophoto) :
- **.png** : Format PNG (sans perte, recommandé)
- **.jpg** : Format JPEG (avec compression, plus petit)
- **.tif** : Format TIFF (haute qualité, géoréférencé possible)

---

## Dépannage

### Le mesh généré est trop grossier
→ Augmentez la **Profondeur Poisson** (essayez 10 ou 12)

### Le traitement est trop lent
→ Réduisez la **Profondeur Poisson** (essayez 6 ou 7)

### L'orthophoto est vide ou incorrecte
→ Vérifiez que les **Coins de l'orthophoto** correspondent bien à la zone de votre nuage de points

### Erreur de mémoire insuffisante
→ Réduisez la **Profondeur Poisson** ou décimez le nuage de points en amont

### Le mesh n'a pas de couleurs
→ Vérifiez que **"Colorier le mesh"** est bien coché et que votre nuage de points source contient des informations de couleur RGB

---

## Support et Contact

Pour toute question ou problème avec ces options, veuillez consulter :
- Le README.md du projet
- Les issues GitHub du dépôt
- La documentation de PCL (Point Cloud Library)
- La documentation de PDAL (Point Data Abstraction Library)

---

**Dernière mise à jour** : Novembre 2025  
**Version du programme** : 2.0
