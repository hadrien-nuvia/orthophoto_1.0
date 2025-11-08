# Orthophoto 2.0

Application de création d'orthophotos à partir de nuages de points 3D (E57, LAS, LAZ) avec génération de mesh par algorithme de Poisson et rendu haute qualité.

## 🎯 Fonctionnalités Principales

### Modes d'Orthophoto Avancés

1. **Orthophoto en Élévation** - Vues verticales alignées sur un axe défini (façades, murs)
2. **Orthophoto en Plan** - Vues horizontales avec coupe à une hauteur définie (toits, sols)
3. **Coins XYZ (Classique)** - Mode boîte englobante traditionnelle
4. **Mode Batch** - Traitement automatique de plusieurs orthophotos en une seule opération

### Traitement du Nuage de Points

- **Filtrage PDAL** : Suppression d'outliers et décimation configurable
- **Génération de Mesh** : Reconstruction de surface par algorithme de Poisson (PCL)
- **Coloration Automatique** : Transfert des couleurs du nuage vers le mesh
- **Résolution Configurable** : Définition en pixels par mètre

### Gestion de Projets

- **Sauvegarde de Travail** : Fichiers .orth pour sauvegarder configurations et paramètres
- **Batch Processing** : Configuration et génération de multiples orthophotos
- **Reproductibilité** : Chargez et modifiez vos configurations sauvegardées

### Formats Supportés

- **Entrée** : E57, LAS, LAZ
- **Sortie** : PNG, JPEG, TIFF
- **Projet** : .orth (format JSON)

## 📚 Documentation

| Document | Description |
|----------|-------------|
| **[OPTIONS.md](OPTIONS.md)** | Guide complet des paramètres et options de l'application |
| **[PC_CONFIGURATION.md](PC_CONFIGURATION.md)** | Configuration matérielle recommandée pour différents cas d'usage |
| **[DEVELOPMENT.md](DEVELOPMENT.md)** | Guide de développement, compilation, et CI/CD |
| **[FORCE_STOP_ACTIONS.md](FORCE_STOP_ACTIONS.md)** | Guide pour forcer l'arrêt des actions GitHub en cours |
| **[GAUSSIAN_SPLATTING.md](GAUSSIAN_SPLATTING.md)** | Planification Phase 2 - Intégration Gaussian Splatting |

## 🚀 Démarrage Rapide

### Pré-requis

- Windows 10/11 64-bit (ou Linux/macOS)
- 16 GB RAM minimum (32 GB recommandé)
- Visual Studio 2019/2022 ou GCC 9+
- CMake 3.16+
- vcpkg (gestionnaire de dépendances)

### Installation

```bash
# 1. Cloner le dépôt
git clone https://github.com/hadrien-nuvia/orthophoto_2.0.git
cd orthophoto_2.0

# 2. Installer les dépendances avec vcpkg
vcpkg install qtbase vtk pcl pdal --triplet x64-windows

# 3. Compiler le projet
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=[chemin-vcpkg]/scripts/buildsystems/vcpkg.cmake
cmake --build . --config Release
```

Pour des instructions détaillées, consultez [DEVELOPMENT.md](DEVELOPMENT.md).

### Utilisation Rapide

1. **Sélectionner un fichier** : E57, LAS ou LAZ
2. **Pré-traiter** : Appliquer les filtres PDAL (outliers, décimation)
3. **Générer le mesh** : Reconstruction de Poisson avec profondeur configurable
4. **Configurer et ajouter au batch** : Définir plusieurs orthophotos avec différents paramètres
5. **Sauvegarder le projet** : Menu Fichier → Enregistrer (.orth)
6. **Traiter le batch** : Générer automatiquement toutes les orthophotos configurées

Consultez [OPTIONS.md](OPTIONS.md) pour un guide détaillé de chaque paramètre et du mode batch.

## 🏗️ Architecture Technique

- **Interface** : Qt6 (Widgets)
- **Visualisation 3D** : VTK 9.x
- **Traitement Points** : PCL 1.12+, PDAL 2.4+
- **Calcul Matriciel** : Eigen3
- **Build System** : CMake + vcpkg

## 💻 Configuration Matérielle

| Profil | CPU | RAM | Cas d'Usage |
|--------|-----|-----|-------------|
| **Minimal** | 4C/8T | 16 GB | Profondeur Poisson ≤ 8, fichiers ≤ 5 GB |
| **Recommandé** | 8C/16T | 32 GB | Profondeur Poisson ≤ 12, fichiers ≤ 10 GB |
| **Optimal** | 16C/32T | 64 GB | Profondeur Poisson ≤ 16, fichiers ≤ 20 GB |

Consultez [PC_CONFIGURATION.md](PC_CONFIGURATION.md) pour des recommandations détaillées.

## 🤝 Contribution

Ce projet utilise GitHub Actions pour l'intégration continue. Consultez [DEVELOPMENT.md](DEVELOPMENT.md) pour :
- Configuration des workflows CI/CD
- Stratégie de cache vcpkg optimisée
- Guide de développement local

## 🔮 Roadmap

### Phase 2 - Gaussian Splatting
Integration prévue du Gaussian Splatting pour des rendus photorealistic à partir de fichiers E57P (avec photos calibrées). Voir [GAUSSIAN_SPLATTING.md](GAUSSIAN_SPLATTING.md) pour les détails.

## 📝 License

Ce projet est sous licence MIT - voir le fichier LICENSE pour plus de détails.

## 🙏 Remerciements

- **PCL** - Point Cloud Library
- **PDAL** - Point Data Abstraction Library
- **VTK** - Visualization Toolkit
- **Qt** - Cross-platform application framework
