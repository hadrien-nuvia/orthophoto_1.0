# Configuration PC Recommandée pour Orthophoto 2.0

Ce document fournit des recommandations de configuration matérielle pour produire des orthophotos à partir de nuages de points, en particulier pour des fichiers de **5 Go**.

## Table des Matières
1. [Résumé Rapide](#résumé-rapide)
2. [Configuration Minimale](#configuration-minimale)
3. [Configuration Recommandée](#configuration-recommandée)
4. [Configuration Optimale](#configuration-optimale)
5. [Détails des Composants](#détails-des-composants)
6. [Temps de Traitement Estimés](#temps-de-traitement-estimés)
7. [Recommandations Spécifiques](#recommandations-spécifiques)

---

## Résumé Rapide

Pour un nuage de points de **5 Go** :

| Composant | Minimum | Recommandé | Optimal |
|-----------|---------|------------|---------|
| **CPU** | 4 cœurs / 8 threads | 6-8 cœurs / 12-16 threads | 8+ cœurs / 16+ threads |
| **RAM** | 16 Go | 32 Go | 64 Go |
| **GPU** | Intégré (pour UI) | GTX 1660 / RTX 3050 | RTX 3060+ / RTX 4060+ |
| **Stockage** | 50 Go SSD | 100 Go NVMe SSD | 250 Go+ NVMe SSD |
| **OS** | Windows 10/11 64-bit | Windows 11 / Linux | Windows 11 / Linux |

---

## Configuration Minimale

### Pour traiter un nuage de points de 5 Go avec Poisson depth ≤ 8

**Processeur (CPU)**
- Intel Core i5-10400 ou AMD Ryzen 5 3600
- 4 cœurs physiques / 8 threads minimum
- Fréquence de base : 2.9 GHz+

**Mémoire (RAM)**
- **16 Go DDR4** minimum
- Fréquence : 2666 MHz ou supérieure
- ⚠️ **Attention** : Avec 16 Go, vous serez limité à une profondeur Poisson ≤ 8-9

**Carte Graphique (GPU)**
- Intel UHD Graphics 630 ou équivalent (intégré)
- AMD Radeon RX 560 ou NVIDIA GTX 1050
- 2 Go VRAM minimum
- Utilisé principalement pour l'interface Qt et la visualisation VTK

**Stockage**
- 50 Go d'espace libre minimum
- SSD SATA recommandé (minimum HDD 7200 RPM)
- 20 Go pour les fichiers temporaires de traitement
- 30 Go pour l'application et les dépendances

**Système d'exploitation**
- Windows 10 64-bit (version 1909 ou supérieure)
- Linux (Ubuntu 20.04+, Debian 11+)

### Limitations de cette configuration
- ❌ Profondeur Poisson limitée à 8-9 maximum
- ❌ Temps de traitement longs (2-4 heures)
- ❌ Risque de manque de mémoire avec des nuages denses
- ❌ Pas de multitâche pendant le traitement

---

## Configuration Recommandée

### Pour traiter un nuage de points de 5 Go avec Poisson depth ≤ 12

**Processeur (CPU)**
- Intel Core i7-12700 ou AMD Ryzen 7 5800X
- 6-8 cœurs physiques / 12-16 threads
- Fréquence de base : 3.6 GHz+
- Cache L3 : 16 Mo minimum

**Mémoire (RAM)**
- **32 Go DDR4/DDR5**
- Fréquence : 3200 MHz (DDR4) ou 4800 MHz (DDR5)
- Configuration dual-channel (2 x 16 Go)
- ✅ Permet d'utiliser une profondeur Poisson jusqu'à 12

**Carte Graphique (GPU)**
- NVIDIA GTX 1660 Super (6 Go VRAM)
- NVIDIA RTX 3050 (8 Go VRAM)
- AMD Radeon RX 6600 (8 Go VRAM)
- Utilisé pour la visualisation et l'interface

**Stockage**
- 100 Go d'espace libre minimum
- **NVMe SSD PCIe 3.0** (vitesse de lecture : 3000 Mo/s+)
- 500 Go recommandé pour plusieurs projets
- Considérer un second disque pour les archives

**Système d'exploitation**
- Windows 11 64-bit (recommandé)
- Linux Ubuntu 22.04 LTS ou Debian 12

### Avantages de cette configuration
- ✅ Profondeur Poisson jusqu'à 12
- ✅ Temps de traitement raisonnables (30 min - 1h30)
- ✅ Multitâche possible pendant le traitement léger
- ✅ Bon équilibre prix/performance

---

## Configuration Optimale

### Pour traiter des nuages de points de 5 Go+ avec Poisson depth ≤ 16+

**Processeur (CPU)**
- Intel Core i9-13900K ou AMD Ryzen 9 7950X
- 8+ cœurs physiques / 16+ threads
- Fréquence boost : 5.0 GHz+
- Cache L3 : 32 Mo+
- Support AVX2/AVX-512 pour PCL optimisé

**Mémoire (RAM)**
- **64 Go DDR5**
- Fréquence : 5200 MHz ou supérieure
- Configuration quad-channel si possible (serveur/HEDT)
- ✅ Permet d'utiliser une profondeur Poisson de 16+ sans problème

**Carte Graphique (GPU)**
- NVIDIA RTX 3060 (12 Go VRAM)
- NVIDIA RTX 4060 Ti (16 Go VRAM)
- NVIDIA RTX 3080/4080 (pour projets futurs avec GPU acceleration)
- AMD Radeon RX 7700 XT (12 Go VRAM)

**Stockage**
- 250 Go+ d'espace libre
- **NVMe SSD PCIe 4.0** (vitesse de lecture : 7000 Mo/s+)
- 1 To recommandé pour plusieurs projets
- Configuration RAID 0 pour performances maximales (optionnel)
- Disque secondaire pour sauvegardes

**Système d'exploitation**
- Windows 11 Pro 64-bit
- Linux Ubuntu 22.04 LTS ou Arch Linux (pour performances maximales)

### Avantages de cette configuration
- ✅ Profondeur Poisson jusqu'à 16-20
- ✅ Temps de traitement rapides (15-45 minutes)
- ✅ Multitâche intensif possible
- ✅ Prêt pour des nuages de points jusqu'à 20 Go+
- ✅ Support pour Gaussian Splatting (développement futur)

---

## Détails des Composants

### Processeur (CPU)

Le CPU est le composant le plus important pour ce logiciel car :
- **Reconstruction de surface Poisson** : Très intensive en CPU, utilise tous les cœurs disponibles
- **Filtrage PDAL** : Traitement parallèle multi-thread
- **Calcul des normales** : Algorithme PCL multi-thread

**Recommandations par usage** :

| Cas d'usage | CPU recommandé | Justification |
|-------------|----------------|---------------|
| Profondeur ≤ 8 | 4C/8T (i5-10400) | Suffisant pour petits projets |
| Profondeur 9-12 | 8C/16T (i7-12700) | Équilibre optimal |
| Profondeur 13-16 | 12C/24T (i9-12900K) | Haute résolution |
| Profondeur 16+ | 16C/32T (Ryzen 9 7950X) | Projets professionnels |

**Caractéristiques importantes** :
- Nombre de cœurs > Fréquence (pour PCL/PDAL)
- Cache L3 large (16+ Mo)
- Support AVX2 minimum, AVX-512 un plus

### Mémoire (RAM)

La RAM est critique car elle stocke :
- Le nuage de points complet en mémoire
- L'octree de Poisson (croissance exponentielle avec la profondeur)
- Le mesh généré
- Les buffers de traitement PDAL

**Estimation de consommation RAM** :

```
RAM requise ≈ (Taille fichier × 3) + (4^profondeur × 100 octets)
```

Pour un fichier de 5 Go :

| Profondeur Poisson | RAM minimum | RAM recommandée |
|-------------------|-------------|-----------------|
| 6-8 | 12 Go | 16 Go |
| 9-10 | 16 Go | 24 Go |
| 11-12 | 24 Go | 32 Go |
| 13-14 | 32 Go | 48 Go |
| 15-16 | 48 Go | 64 Go |
| 17+ | 64 Go+ | 128 Go+ |

**Caractéristiques importantes** :
- Dual-channel obligatoire (2 barrettes)
- Fréquence 3200 MHz minimum pour DDR4
- Latence CAS 16 ou inférieure

### Carte Graphique (GPU)

**Usage actuel** :
- Visualisation VTK (nuage de points et mesh)
- Rendu de l'interface Qt
- Export d'orthophoto (rendering)

**Usage futur prévu** :
- Gaussian Splatting (voir GAUSSIAN_SPLATTING.md)
- Accélération GPU pour certains filtres PDAL

**Recommandations** :

| Budget | GPU | VRAM | Commentaire |
|--------|-----|------|-------------|
| Économique | GTX 1650 | 4 Go | Visualisation basique |
| Standard | GTX 1660 Super | 6 Go | Bon équilibre |
| Recommandé | RTX 3050 | 8 Go | Support ray tracing |
| Performance | RTX 3060 | 12 Go | Prêt pour le futur |
| Professionnel | RTX 4070 | 12 Go | Maximum de performance |

⚠️ **Note** : Le GPU n'est pas critique pour les performances actuelles. Privilégiez CPU et RAM.

### Stockage

**Besoins en espace** :

| Type de fichier | Espace requis (pour 5 Go de nuage) |
|-----------------|-------------------------------------|
| Fichier source (.e57/.las/.laz) | 5 Go |
| Fichier PDAL temporaire | 5-8 Go |
| Mesh Poisson (depth 8) | 500 Mo - 1 Go |
| Mesh Poisson (depth 12) | 2-4 Go |
| Orthophoto (haute résolution) | 100-500 Mo |
| **Total workspace** | **15-25 Go par projet** |

**Performance du stockage** :

| Type | Vitesse lecture | Vitesse écriture | Impact sur temps |
|------|----------------|------------------|------------------|
| HDD 7200 RPM | 150 Mo/s | 150 Mo/s | Baseline (+100%) |
| SSD SATA | 550 Mo/s | 520 Mo/s | +30% plus rapide |
| NVMe PCIe 3.0 | 3500 Mo/s | 3000 Mo/s | +50% plus rapide |
| NVMe PCIe 4.0 | 7000 Mo/s | 5000 Mo/s | +60% plus rapide |

**Recommandations** :
- Minimum : SSD SATA 250 Go
- Recommandé : NVMe PCIe 3.0 500 Go
- Optimal : NVMe PCIe 4.0 1 To
- Toujours garder 20% d'espace libre pour performances optimales

---

## Temps de Traitement Estimés

### Pour un nuage de points de 5 Go (environ 100 millions de points)

#### Configuration Minimale (i5-10400, 16 Go RAM)

| Étape | Profondeur 6 | Profondeur 8 |
|-------|--------------|--------------|
| Pré-traitement PDAL | 3-5 min | 3-5 min |
| Calcul des normales | 5-8 min | 5-8 min |
| Reconstruction Poisson | 8-12 min | 25-35 min |
| Coloration du mesh | 3-5 min | 5-8 min |
| Export orthophoto | 2-3 min | 3-5 min |
| **Total** | **21-33 min** | **41-61 min** |

#### Configuration Recommandée (i7-12700, 32 Go RAM)

| Étape | Profondeur 8 | Profondeur 10 | Profondeur 12 |
|-------|--------------|---------------|---------------|
| Pré-traitement PDAL | 2-3 min | 2-3 min | 2-3 min |
| Calcul des normales | 3-4 min | 3-4 min | 3-4 min |
| Reconstruction Poisson | 10-15 min | 20-30 min | 45-60 min |
| Coloration du mesh | 2-3 min | 3-5 min | 5-8 min |
| Export orthophoto | 2-3 min | 2-3 min | 3-5 min |
| **Total** | **19-28 min** | **30-45 min** | **58-80 min** |

#### Configuration Optimale (i9-13900K, 64 Go RAM)

| Étape | Profondeur 10 | Profondeur 12 | Profondeur 14 | Profondeur 16 |
|-------|---------------|---------------|---------------|---------------|
| Pré-traitement PDAL | 1-2 min | 1-2 min | 1-2 min | 1-2 min |
| Calcul des normales | 2-3 min | 2-3 min | 2-3 min | 2-3 min |
| Reconstruction Poisson | 8-12 min | 20-25 min | 40-50 min | 80-100 min |
| Coloration du mesh | 2-3 min | 3-4 min | 4-6 min | 6-8 min |
| Export orthophoto | 1-2 min | 2-3 min | 3-4 min | 4-5 min |
| **Total** | **14-22 min** | **28-37 min** | **50-65 min** | **93-118 min** |

### Facteurs qui influencent le temps de traitement

**Augmente le temps** :
- ⬆️ Densité du nuage de points
- ⬆️ Profondeur Poisson
- ⬆️ Résolution de l'orthophoto (pixels par mètre)
- ⬆️ Activation du coloriage du mesh
- ⬆️ Filtres PDAL activés (outliers)

**Réduit le temps** :
- ⬇️ Décimation du nuage de points (filtre PDAL)
- ⬇️ Profondeur Poisson réduite
- ⬇️ Résolution de l'orthophoto réduite
- ⬇️ Stockage NVMe rapide
- ⬇️ Plus de cœurs CPU

---

## Recommandations Spécifiques

### Pour les débutants / Usage occasionnel

**Budget** : 1000-1500 €

```
CPU: AMD Ryzen 5 5600X (6C/12T)
RAM: 32 Go DDR4 3200 MHz (2x16 Go)
GPU: GTX 1650 ou intégré
SSD: 500 Go NVMe PCIe 3.0
Carte mère: B450/B550 avec 4 slots RAM
Alimentation: 550W 80+ Bronze
```

**Cas d'usage** :
- Projets occasionnels
- Profondeur Poisson ≤ 10
- Fichiers jusqu'à 5-8 Go

### Pour les professionnels / Usage régulier

**Budget** : 2000-3000 €

```
CPU: Intel Core i7-13700K (16C/24T)
RAM: 64 Go DDR5 5200 MHz (2x32 Go)
GPU: RTX 3060 12 Go ou RTX 4060
SSD: 1 To NVMe PCIe 4.0 (Samsung 980 Pro)
Carte mère: Z690/Z790 avec 4 slots RAM
Alimentation: 750W 80+ Gold
Refroidissement: Tour CPU haute performance
```

**Cas d'usage** :
- Usage quotidien
- Profondeur Poisson ≤ 14
- Fichiers jusqu'à 20 Go
- Multiples projets simultanés

### Pour les studios / Production intensive

**Budget** : 4000-6000 €

```
CPU: AMD Ryzen 9 7950X (16C/32T) ou Threadripper
RAM: 128 Go DDR5 5600 MHz (4x32 Go)
GPU: RTX 4070 Ti 12 Go ou RTX 4080
SSD 1: 2 To NVMe PCIe 4.0 (travail)
SSD 2: 2 To NVMe PCIe 4.0 (archives)
Carte mère: X670E avec 4+ slots RAM
Alimentation: 1000W 80+ Platinum
Refroidissement: AIO 360mm ou watercooling custom
```

**Cas d'usage** :
- Production intensive
- Profondeur Poisson ≤ 18
- Fichiers jusqu'à 50 Go
- Projets multiples simultanés
- Prêt pour Gaussian Splatting

---

## Optimisations Logicielles

### Paramètres Windows

```powershell
# Désactiver l'indexation sur le disque de travail
# Panneau de configuration > Options d'indexation

# Augmenter la mémoire virtuelle
# Système > Paramètres système avancés > Performances > Avancé
# Définir : Taille initiale = RAM × 1.5
#          Taille maximale = RAM × 3
```

### Paramètres Linux

```bash
# Augmenter les limites de mémoire
sudo sysctl -w vm.max_map_count=262144
sudo sysctl -w vm.overcommit_memory=1

# Pour rendre permanent
echo "vm.max_map_count=262144" | sudo tee -a /etc/sysctl.conf
echo "vm.overcommit_memory=1" | sudo tee -a /etc/sysctl.conf
```

### Optimisation de l'application

**Avant le traitement** :
- Fermer toutes les applications non essentielles
- Désactiver les antivirus en temps réel (temporairement)
- Vérifier que le disque a 20% d'espace libre
- Utiliser le filtre de décimation pour les tests (1 point sur 10)

**Pendant le traitement** :
- Ne pas utiliser d'autres applications lourdes
- Surveiller l'utilisation de la RAM (Gestionnaire des tâches)
- Surveiller la température du CPU (HWMonitor, CoreTemp)

---

## FAQ

### Q : Mon PC a 16 Go de RAM, puis-je traiter un fichier de 5 Go ?

**R :** Oui, mais avec des limitations :
- Profondeur Poisson maximum : 8-9
- Risque de manque de mémoire si d'autres applications sont ouvertes
- Utilisez le filtre de décimation si nécessaire
- Fermez toutes les applications non essentielles

### Q : Quelle profondeur Poisson dois-je utiliser ?

**R :** Cela dépend de vos besoins :
- **Prévisualisation rapide** : 6-7 (5-10 minutes)
- **Qualité standard** : 8-9 (15-30 minutes)
- **Haute qualité** : 10-12 (30-90 minutes)
- **Très haute qualité** : 13-15 (1-3 heures, nécessite 48+ Go RAM)
- **Qualité maximale** : 16+ (3+ heures, nécessite 64+ Go RAM)

### Q : Le GPU est-il important ?

**R :** Actuellement, non. Le GPU est utilisé uniquement pour :
- L'interface utilisateur Qt
- La visualisation VTK
- Le rendu de l'orthophoto finale

Un GPU intégré ou d'entrée de gamme suffit. Cependant, pour les développements futurs (Gaussian Splatting), un GPU dédié avec 8+ Go de VRAM sera recommandé.

### Q : SSD ou HDD ?

**R :** **SSD fortement recommandé**, NVMe encore mieux :
- SSD SATA : 30-50% plus rapide qu'un HDD
- NVMe PCIe 3.0 : 50-60% plus rapide qu'un HDD
- La différence est particulièrement visible lors du chargement des fichiers et de l'écriture du mesh

### Q : Puis-je utiliser un ordinateur portable ?

**R :** Oui, avec certaines considérations :
- Choisir un modèle avec refroidissement efficace
- Vérifier que la RAM est extensible (pas soudée)
- Préférer des modèles gaming/workstation
- Surveiller les températures lors des traitements longs
- Recommandations :
  - Lenovo ThinkPad P Series
  - Dell Precision Mobile
  - HP ZBook
  - ASUS ProArt StudioBook

### Q : Combien de temps garder mon PC ?

**R :** Durée de vie estimée par configuration :
- **Configuration minimale** : 2-3 ans avant obsolescence pour ce logiciel
- **Configuration recommandée** : 4-5 ans, évolutif (RAM extensible)
- **Configuration optimale** : 6-8 ans, prêt pour les développements futurs

### Q : Quels composants prioriser avec un budget limité ?

**R :** Ordre de priorité :
1. **RAM** (32 Go minimum pour la plupart des cas)
2. **CPU** (8 cœurs / 16 threads minimum)
3. **SSD NVMe** (500 Go minimum)
4. **GPU** (peut être basique, upgrade plus tard)

Stratégie : Acheter une carte mère avec slots RAM libres pour upgrade futur.

---

## Matrice de Décision Rapide

Utilisez cette matrice pour choisir votre configuration :

| Votre situation | Configuration recommandée |
|-----------------|---------------------------|
| Budget serré, usage occasionnel, petits projets | **Minimale** |
| Budget moyen, usage régulier, projets standards | **Recommandée** |
| Budget flexible, usage professionnel, gros projets | **Optimale** |
| Nuage de points < 2 Go | **Minimale** suffit |
| Nuage de points 2-8 Go | **Recommandée** |
| Nuage de points 8-20 Go | **Optimale** |
| Profondeur Poisson ≤ 8 | **Minimale** suffit |
| Profondeur Poisson 9-12 | **Recommandée** |
| Profondeur Poisson 13+ | **Optimale** obligatoire |
| Je veux tester l'application | **Minimale** |
| Je vais l'utiliser pour mon travail | **Recommandée** |
| Je suis un studio professionnel | **Optimale** |

---

## Configurations PC Complètes Clé en Main

### Configuration "Starter" - 1200 €

```
Processeur: AMD Ryzen 5 5600X
Carte mère: MSI B550-A PRO
RAM: Corsair Vengeance 32 Go (2×16 Go) DDR4 3200 MHz
GPU: Intégré ou GTX 1650
SSD: Samsung 970 EVO Plus 500 Go NVMe
Alimentation: Corsair CV550 550W 80+ Bronze
Boîtier: Fractal Design Meshify C
Refroidissement: Stock AMD ou be quiet! Pure Rock 2
```

**Performance attendue** : Poisson depth ≤ 10, temps de traitement 30-60 min pour 5 Go

### Configuration "Pro" - 2500 €

```
Processeur: Intel Core i7-13700K
Carte mère: ASUS TUF Gaming Z690-PLUS
RAM: G.Skill Trident Z5 64 Go (2×32 Go) DDR5 5200 MHz
GPU: NVIDIA RTX 3060 12 Go
SSD: Samsung 980 Pro 1 To NVMe PCIe 4.0
Alimentation: Corsair RM750x 750W 80+ Gold
Boîtier: be quiet! Silent Base 802
Refroidissement: Noctua NH-D15 ou NZXT Kraken X63
```

**Performance attendue** : Poisson depth ≤ 14, temps de traitement 20-45 min pour 5 Go

### Configuration "Studio" - 5000 €

```
Processeur: AMD Ryzen 9 7950X
Carte mère: ASUS ROG Crosshair X670E Hero
RAM: G.Skill Trident Z5 RGB 128 Go (4×32 Go) DDR5 6000 MHz
GPU: NVIDIA RTX 4070 Ti 12 Go
SSD 1: Samsung 990 Pro 2 To NVMe PCIe 4.0 (travail)
SSD 2: Samsung 980 Pro 2 To NVMe PCIe 4.0 (backup)
Alimentation: Corsair HX1000i 1000W 80+ Platinum
Boîtier: Fractal Design Define 7 XL
Refroidissement: Arctic Liquid Freezer II 360mm
```

**Performance attendue** : Poisson depth ≤ 18, temps de traitement 15-30 min pour 5 Go

---

## Support et Mises à Jour

Ce document est basé sur :
- **Version du logiciel** : Orthophoto & Mesh Tool 2.0
- **Date** : Novembre 2024
- **Algorithme** : Poisson Surface Reconstruction (PCL)
- **Bibliothèques** : PCL 1.12+, PDAL 2.4+, VTK 9.1+

Pour toute question sur les configurations matérielles, consultez :
- Le README.md du projet
- Le fichier OPTIONS.md pour les paramètres d'optimisation
- Les issues GitHub pour les retours d'expérience communautaires

---

**Note finale** : Ces recommandations sont basées sur des estimations et des tests. Les performances réelles peuvent varier selon la densité du nuage de points, sa distribution spatiale, et les paramètres choisis. Il est toujours recommandé de commencer avec une profondeur Poisson faible (6-8) pour évaluer les performances de votre système avant d'augmenter progressivement.
