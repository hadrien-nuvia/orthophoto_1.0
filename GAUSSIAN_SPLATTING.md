# Gaussian Splatting Integration (Phase 2)

## Objectif

Ajouter la capacité de faire du Gaussian Splatting en utilisant les exports photo compris dans un fichier .E57P, avec le nuage de points comme référence.

## Contexte

Le Gaussian Splatting est une technique de rendu 3D récente qui permet de créer des représentations photorealistic de scènes 3D à partir de photos et de positions de caméra. Cette technique est particulièrement adaptée aux données de scan laser avec photos calibrées.

## Format E57P

Le format .E57P (E57 avec photos) est une extension du format E57 standard qui inclut :
- Le nuage de points 3D
- Les images photographiques calibrées
- Les paramètres de caméra (position, orientation, distorsion)
- Les métadonnées de calibration

## Architecture Proposée

### 1. Extraction des Données E57P

```cpp
class E57PReader {
public:
    struct CameraData {
        std::string imagePath;
        Eigen::Matrix4d pose;           // Position et orientation
        Eigen::Matrix3d intrinsics;     // Paramètres intrinsèques
        std::vector<double> distortion; // Paramètres de distorsion
    };
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;
    std::vector<CameraData> cameras;
    
    bool loadE57P(const std::string& filename);
    void extractImages(const std::string& outputDir);
};
```

### 2. Intégration Gaussian Splatting

Plusieurs options sont disponibles :

#### Option A : Utiliser une bibliothèque externe
- **gsplat** (CUDA) - https://github.com/nerfstudio-project/gsplat
- **diff-gaussian-rasterization** - https://github.com/graphdeco-inria/diff-gaussian-rasterization
- **Avantages** : Performance optimisée, implémentation mature
- **Inconvénients** : Dépendance CUDA, complexité d'intégration

#### Option B : Implémentation CPU simplifiée
- Rendu basique sans optimisation GPU
- Suffisant pour des visualisations de taille moyenne
- Plus facile à intégrer dans le code existant

### 3. Pipeline de Traitement

```
E57P File → Extract Photos + Point Cloud
           ↓
    Preprocess Images (Undistort)
           ↓
    Initialize Gaussians from Point Cloud
           ↓
    Optimize Gaussians using Photos
           ↓
    Render Novel Views / Export
```

### 4. Interface Utilisateur

Ajouter un nouvel onglet "Gaussian Splatting" avec :

```cpp
QGroupBox *gaussianSplattingGroup = new QGroupBox("Gaussian Splatting", this);
- QPushButton *loadE57PBtn : Charger fichier E57P
- QPushButton *extractPhotosBtn : Extraire les photos
- QPushButton *initGaussiansBtn : Initialiser les Gaussiens depuis le nuage
- QSpinBox *numIterationsSpinBox : Nombre d'itérations d'optimisation
- QProgressBar *trainingProgressBar : Progrès de l'entraînement
- QPushButton *renderViewBtn : Rendre une vue
- QPushButton *exportModelBtn : Exporter le modèle
```

## Dépendances Additionnelles

### Bibliothèques Requises

1. **E57 Format** (déjà utilisé via PDAL)
   - `libe57format` pour la lecture détaillée

2. **Image Processing**
   - OpenCV pour le traitement d'images
   - Eigen pour les calculs matriciels (déjà dans PCL)

3. **Gaussian Splatting**
   - Option A : CUDA + gsplat
   - Option B : Implémentation CPU custom

### Mise à jour vcpkg.json

```json
{
  "name": "orthophoto",
  "version": "0.2.0",
  "dependencies": [
    "vtk",
    "pcl",
    "pdal",
    "opencv",
    "eigen3",
    "e57format"
  ]
}
```

### Mise à jour CMakeLists.txt

```cmake
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(E57Format REQUIRED)

target_link_libraries(orthophoto_gui PRIVATE
    Qt6::Widgets
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
    PDAL::PDAL
    ${OpenCV_LIBS}
    Eigen3::Eigen
    E57Format::E57Format
)
```

## Étapes de Développement

### Phase 2.1 : Préparation
- [ ] Ajouter support de lecture E57P
- [ ] Extraire et afficher les photos
- [ ] Visualiser les positions de caméra dans le nuage de points

### Phase 2.2 : Initialisation
- [ ] Convertir le nuage de points en Gaussiens 3D
- [ ] Implémenter l'initialisation des paramètres (position, covariance, couleur)

### Phase 2.3 : Optimisation
- [ ] Implémenter ou intégrer un optimiseur de Gaussiens
- [ ] Ajouter la fonction de perte basée sur les photos
- [ ] Implémenter l'interface d'entraînement

### Phase 2.4 : Rendu
- [ ] Implémenter le rasterizer de Gaussiens
- [ ] Ajouter l'export de vues rendues
- [ ] Implémenter l'export du modèle

### Phase 2.5 : Optimisation
- [ ] Optimiser les performances
- [ ] Ajouter le support GPU si disponible
- [ ] Implémenter le LOD (Level of Detail)

## Ressources et Références

### Papers
- "3D Gaussian Splatting for Real-Time Radiance Field Rendering" (SIGGRAPH 2023)
  https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/

### Code
- Original Implementation: https://github.com/graphdeco-inria/gaussian-splatting
- gsplat Library: https://github.com/nerfstudio-project/gsplat
- Taichi Implementation (CPU/GPU): https://github.com/wanmeihuali/taichi_3d_gaussian_splatting

### Tutorials
- https://github.com/MrNeRF/awesome-3D-gaussian-splatting
- https://huggingface.co/blog/gaussian-splatting

## Considérations Techniques

### Performance
- **GPU recommandé** : NVIDIA avec CUDA 11.8+
- **RAM** : Minimum 16 GB, recommandé 32 GB
- **VRAM** : Minimum 8 GB pour des scènes moyennes

### Limitations
- Les fichiers E57P ne sont pas tous identiques - format peut varier
- La calibration des caméras doit être précise
- Le temps d'entraînement peut être long (quelques minutes à quelques heures)

### Alternatives pour CPU uniquement
Si pas de GPU disponible :
1. Utiliser des Gaussiens 2D projetés (plus rapide)
2. Réduire la résolution du modèle
3. Utiliser un rasterizer CPU optimisé (ex: avec Intel TBB)

## Timeline Estimée

- **Recherche et prototypage** : 1-2 semaines
- **Implémentation de base** : 2-3 semaines
- **Optimisation et tests** : 1-2 semaines
- **Documentation** : 1 semaine

**Total** : 5-8 semaines pour une implémentation complète

## Notes

Cette fonctionnalité est ambitieuse et représente un ajout majeur à l'application. Elle peut être développée de manière incrémentale :

1. Commencer par l'extraction et la visualisation des données E57P
2. Implémenter un viewer simple des photos avec positions
3. Ajouter progressivement le Gaussian Splatting

Une approche hybride est également possible : utiliser le Gaussian Splatting pour améliorer la qualité des orthophotos existantes plutôt que comme fonctionnalité autonome.
