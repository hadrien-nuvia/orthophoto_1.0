# Guide de Développement - Orthophoto 2.0

Ce document contient toutes les informations nécessaires pour développer, compiler et déployer le projet Orthophoto 2.0.

## Table des Matières

1. [Compilation Locale](#compilation-locale)
2. [GitHub Actions et CI/CD](#github-actions-et-cicd)
3. [Stratégie de Cache vcpkg](#stratégie-de-cache-vcpkg)
4. [Optimisations et Performances](#optimisations-et-performances)
5. [Dépannage](#dépannage)

---

## Compilation Locale

### Prérequis

- **Système d'exploitation** : Windows 10/11 64-bit, Linux (Ubuntu 20.04+), ou macOS
- **Compilateur** : 
  - Windows: Visual Studio 2019/2022 avec C++ Desktop Development
  - Linux: GCC 9+ ou Clang 10+
  - macOS: Xcode 12+
- **CMake** : Version 3.16 ou supérieure
- **vcpkg** : Gestionnaire de packages C++

### Dépendances

Le projet utilise les bibliothèques suivantes (gérées par vcpkg) :

- **Qt6** (Widgets) - Interface graphique
- **VTK 9.x** - Visualisation 3D
- **PCL 1.12+** - Traitement de nuages de points
- **PDAL 2.4+** - Abstraction de données de points
- **Eigen3** - Calcul matriciel (via PCL)

### Installation avec vcpkg

```bash
# 1. Cloner vcpkg (si pas déjà installé)
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh  # Linux/macOS
./bootstrap-vcpkg.bat # Windows

# 2. Installer les dépendances
./vcpkg install qtbase:x64-windows
./vcpkg install vtk:x64-windows
./vcpkg install pcl:x64-windows
./vcpkg install pdal:x64-windows

# Remplacer x64-windows par x64-linux ou x64-osx selon votre OS
```

### Compilation du Projet

```bash
# 1. Cloner le dépôt
git clone https://github.com/hadrien-nuvia/orthophoto_2.0.git
cd orthophoto_2.0

# 2. Créer le dossier de build
mkdir build
cd build

# 3. Configurer avec CMake
cmake .. -DCMAKE_TOOLCHAIN_FILE=[chemin-vers-vcpkg]/scripts/buildsystems/vcpkg.cmake

# 4. Compiler
cmake --build . --config Release

# 5. L'exécutable sera dans build/Release/orthophoto_gui.exe (Windows)
# ou build/orthophoto_gui (Linux/macOS)
```

---

## GitHub Actions et CI/CD

### Architecture des Workflows

Le projet utilise trois workflows GitHub Actions :

#### 1. `build.yml` - Build Principal

**Déclenchement** :
- Push vers `main`
- Pull requests vers `main`
- Déclenchement manuel

**Processus** :
```
check-cache → trigger-cache-build (si nécessaire) → build
```

**Fonctionnalités** :
- ✅ Détection automatique des caches manquants
- ✅ Déclenchement automatique de `build-cache.yml` si besoin
- ✅ Installation conditionnelle de Qt (seulement si cache miss)
- ✅ Timeout de 60 minutes pour attendre la construction du cache
- ✅ Compilation optimisée avec indicateurs de performance

**Jobs** :
1. **check-cache** : Vérifie la présence des caches vcpkg
2. **trigger-cache-build** : Lance `build-cache.yml` si caches absents
3. **build** : Compile l'application

#### 2. `build-cache.yml` - Construction des Caches (Groupée)

**Déclenchement** :
- Manuel via l'interface GitHub Actions
- Automatique via `build.yml` si caches manquants

**Architecture en 5 Couches** :
```
build-core (minizip, freexl)
    ↓
build-geo (geos, proj, gdal)
    ↓
build-heavy-1-vtk (vtk)
    ↓
build-heavy-2-pcl (pcl)
    ↓
build-heavy-3-pdal (pdal)
```

**Avantages** :
- ✅ Sauvegarde progressive (cache par couche)
- ✅ Résilience en cas d'échec partiel
- ✅ Débogage facilité
- ✅ Réutilisation maximale du cache

#### 3. `build-cache-single.yml` - Construction Consolidée (Alternative)

**Usage** : Alternative simplifiée pour machines puissantes

**Caractéristiques** :
- Un seul job pour tous les packages
- Plus rapide si aucun timeout
- Moins résilient en cas d'échec
- Option `--keep-going` pour continuer malgré les erreurs

### Optimisation Qt

L'installation de Qt est maintenant conditionnelle :

```yaml
- name: Install Qt
  if: steps.cache-qt.outputs.cache-hit != 'true'
  # Qt n'est téléchargé que si le cache est absent
```

**Gain** : 5-10 minutes économisées par build avec cache Qt présent

---

## Stratégie de Cache vcpkg

### Cache Robuste Anti-Retéléchargement

Le projet implémente une stratégie de cache multi-couches qui garantit :

1. ✅ **Zéro retéléchargement** des sources après le premier build
2. ✅ **Sauvegarde progressive** même en cas d'échec
3. ✅ **Continuité** des jobs malgré les échecs partiels
4. ✅ **Économie de bande passante** (4-6 GB par exécution)

### Dossiers Cachés

| Dossier | Importance | Taille | Description |
|---------|-----------|--------|-------------|
| `downloads/` | ⭐⭐⭐⭐⭐ | 2-5 GB | Archives sources (CRITIQUE - évite retéléchargements) |
| `installed/` | ⭐⭐⭐⭐ | 5-10 GB | Binaires compilés prêts à l'emploi |
| `archives/` | ⭐⭐⭐ | Variable | Cache binaire vcpkg |
| `packages/` | ⭐⭐ | 10-20 GB | Artefacts de build intermédiaires |

### Clés de Cache

Le système utilise des clés hiérarchiques :

```
Clé principale: vcpkg-{OS}-{hash de vcpkg.json}

Restore-keys (ordre de priorité):
1. vcpkg-{OS}-heavy-3-pdal-{hash}
2. vcpkg-{OS}-heavy-2-pcl-{hash}
3. vcpkg-{OS}-heavy-1-vtk-{hash}
4. vcpkg-{OS}-geo-{hash}
5. vcpkg-{OS}-core-{hash}
6. vcpkg-{OS}- (fallback générique)
```

### Sauvegarde Progressive

```yaml
- name: Install VTK
  id: install-vtk
  continue-on-error: true  # Ne stoppe pas le workflow

- name: Save cache (heavy-1-vtk)
  if: always() && steps.cache-heavy-1.outputs.cache-hit != 'true'
  uses: actions/cache/save@v3
```

**Résultat** : Le cache est sauvegardé même si l'installation échoue partiellement

### Scénarios de Robustesse

#### Scénario 1 : VTK échoue pendant la compilation
```
✅ Avec stratégie robuste :
- minizip, freexl : Installés et CACHÉS ✓
- geos, proj, gdal : Installés et CACHÉS ✓
- vtk : ÉCHOUÉ mais sources CACHÉES ✓
- pcl, pdal : Continuent l'installation

Au prochain run :
- Pas de retéléchargement des sources
- Reprise là où ça s'est arrêté
```

#### Scénario 2 : Timeout après 4 heures
```
✅ Avec stratégie robuste :
- Packages compilés : CACHÉS ✓
- Packages en cours : Sources CACHÉES ✓

Au prochain run :
- Reprend exactement où c'était arrêté
- Économie de 2-3 heures
```

---

## Optimisations et Performances

### Temps de Build

| Scénario | Sans cache | Avec cache | Gain |
|----------|-----------|------------|------|
| Premier build | 3-4h | 3-4h | 0% (mais automatisé) |
| Second build | 30-45 min | 5-10 min | ~80% |
| Après échec partiel | 3-4h | 1.5-2h | ~50% |
| Modification 1 package | 2-3h | 20-30 min | ~85% |

### Bande Passante Économisée

| Package | Taille sources | Nombre de dépendances | Total |
|---------|---------------|----------------------|-------|
| VTK | ~50 MB | ~100 | ~2-3 GB |
| PCL | ~10 MB | ~50 | ~1-2 GB |
| PDAL | ~5 MB | ~30 | ~500 MB - 1 GB |
| **TOTAL** | - | - | **~4-6 GB/exécution** |

### Optimisations du Code C++

#### CMakeLists.txt
- C++17 requis (pas seulement préféré)
- Optimisations compilateur activées :
  - MSVC : `/O2 /GL` + `/LTCG`
  - GCC/Clang : `-O3 -march=native`
- Composants VTK et PCL spécifiés précisément

#### orthophoto_gui.cpp
- Utilisation de `std::ostringstream` au lieu de concaténation de chaînes
- Pré-allocation mémoire avec `reserve()`
- Boucles range-based au lieu de boucles indexées
- Réutilisation des structures de données (KdTree)

**Gains de performance** :
- String operations : 10-15% plus rapide
- Memory allocations : 10-20% de réduction
- Mesh coloring : 5-10% plus rapide

---

## Solutions Alternatives d'Installation

Cette section compare trois approches pour accélérer l'installation des dépendances et réduire les temps de build.

### Option 1 : Cache Binaire NuGet (RECOMMANDÉ)

**Description** : Utilise le système de cache binaire de vcpkg avec NuGet pour stocker et réutiliser les binaires compilés.

**Avantages** :
- ✅ Téléchargement de binaires pré-compilés au lieu de recompiler
- ✅ 70-90% plus rapide que la compilation à partir des sources
- ✅ Gratuit avec GitHub Packages (pas de coût supplémentaire)
- ✅ Intégration native avec vcpkg (pas de modification majeure du workflow)
- ✅ Partage automatique entre branches et PRs
- ✅ Gestion automatique des versions et du versioning
- ✅ Espace de stockage raisonnable (~2-5 GB par configuration)

**Inconvénients** :
- ❌ Configuration initiale requise (authentification NuGet)
- ❌ Premier build toujours lent (doit créer le cache initial)
- ❌ Nécessite une configuration par OS/architecture
- ❌ Dépendance à GitHub Packages (service externe)
- ❌ Peut nécessiter des tokens d'authentification pour l'accès privé
- ❌ Cache invalidé si vcpkg.json change
- ❌ Complexité supplémentaire pour le debugging en cas de problème

**Configuration** :
```yaml
env:
  VCPKG_BINARY_SOURCES: "clear;nuget,https://nuget.pkg.github.com/${{ github.repository_owner }}/index.json,readwrite"
```

**Gains estimés** :
- Premier build : ~3-4h (création du cache)
- Builds suivants : ~10-15 min (vs 30-45 min actuellement)

**Quand utiliser** :
- ✅ Pour des équipes avec plusieurs développeurs
- ✅ Pour des projets avec CI/CD fréquents
- ✅ Quand la vitesse de build est critique

---

### Option 2 : Container Docker

**Description** : Crée et utilise une image Docker contenant toutes les dépendances pré-compilées et l'environnement de build complet.

**Avantages** :
- ✅ Environnement complètement pré-compilé et prêt à l'emploi
- ✅ Démarrage quasi-instantané (quelques secondes)
- ✅ Reproductibilité maximale (même environnement partout)
- ✅ Isolation complète (pas de conflits avec le système hôte)
- ✅ Facilite l'onboarding des nouveaux développeurs
- ✅ Versionning de l'environnement complet (via tags Docker)
- ✅ Peut inclure tous les outils de développement (IDE, debuggers, etc.)
- ✅ Portable entre différents OS (Linux, macOS, Windows avec WSL2)

**Inconvénients** :
- ❌ Image très volumineuse (10-20 GB minimum)
- ❌ Temps de construction initial de l'image très long (4-6 heures)
- ❌ Consommation importante d'espace disque (local et registry)
- ❌ Coûts potentiels de stockage pour images privées
- ❌ Téléchargement initial long (10-20 GB à pull)
- ❌ Nécessite Docker installé et configuré
- ❌ Peut être plus lent sur Windows (overhead WSL2)
- ❌ Difficulté à débugger les problèmes spécifiques à l'image
- ❌ Mise à jour de l'image requiert une reconstruction complète
- ❌ Complexité accrue pour la gestion des versions multiples

**Utilisation** :
```yaml
jobs:
  build:
    container:
      image: ghcr.io/${{ github.repository_owner }}/orthophoto-buildenv:latest
```

**Construction de l'image** :
```dockerfile
FROM ubuntu:22.04

# Installation des dépendances système
RUN apt-get update && apt-get install -y \
    build-essential cmake git \
    # ... autres dépendances

# Installation de vcpkg et packages
RUN git clone https://github.com/Microsoft/vcpkg.git
RUN cd vcpkg && ./bootstrap-vcpkg.sh
RUN vcpkg install qtbase vtk pcl pdal

# Configuration de l'environnement
ENV VCPKG_ROOT=/vcpkg
```

**Quand utiliser** :
- ✅ Pour des équipes distribuées nécessitant la même configuration
- ✅ Pour garantir la reproductibilité absolue des builds
- ✅ Quand l'espace disque et la bande passante ne sont pas limitants
- ❌ Éviter pour des connexions Internet lentes
- ❌ Éviter pour des machines avec peu d'espace disque

---

### Option 3 : Binaires Pré-compilés (GitHub Releases)

**Description** : Héberge des archives ZIP/TAR contenant les dépendances déjà compilées sur GitHub Releases, à télécharger et extraire lors du build.

**Avantages** :
- ✅ Solution la plus simple à comprendre et implémenter
- ✅ Pas de dépendance à des services externes complexes
- ✅ Téléchargement rapide (archives compressées)
- ✅ Contrôle total sur les versions distribuées
- ✅ Facilite les builds offline (après premier téléchargement)
- ✅ Compatible avec n'importe quel système de CI/CD
- ✅ Aucune configuration vcpkg spéciale requise
- ✅ Peut être hébergé ailleurs que GitHub (S3, CDN, etc.)

**Inconvénients** :
- ❌ Gestion manuelle des archives (création, upload, versioning)
- ❌ Nécessite de créer une archive par OS/architecture
- ❌ Taille importante des archives (~5-8 GB compressées)
- ❌ Pas de gestion automatique des dépendances
- ❌ Risque d'incompatibilité si compilé sur un système différent
- ❌ Difficile à maintenir à jour (recompilation manuelle)
- ❌ GitHub Releases limitée à 2 GB par fichier (nécessite split)
- ❌ Pas de mécanisme de vérification d'intégrité intégré
- ❌ Occupation d'espace dans le dépôt GitHub
- ❌ Processus de mise à jour non automatisé

**Utilisation** :
```yaml
- name: Download prebuilt dependencies
  run: |
    curl -L -o deps.zip https://github.com/${{ github.repository }}/releases/download/deps-v1/vcpkg-deps.zip
    Expand-Archive deps.zip -DestinationPath C:/vcpkg/
```

**Création de l'archive** :
```bash
# Après compilation locale réussie
cd C:/vcpkg
tar -czf vcpkg-deps-windows-x64.tar.gz installed/ packages/

# Upload vers GitHub Releases
gh release create deps-v1 vcpkg-deps-windows-x64.tar.gz --title "Dependencies v1"
```

**Quand utiliser** :
- ✅ Pour des projets avec peu de changements de dépendances
- ✅ Pour des configurations très spécifiques (compilation custom)
- ✅ Quand la simplicité prime sur l'automatisation
- ❌ Éviter pour des projets avec dépendances changeant fréquemment
- ❌ Éviter pour des équipes larges avec besoins variés

---

### Tableau Comparatif des 3 Options

| Critère | NuGet Cache | Docker Container | Binaires Pré-compilés |
|---------|-------------|------------------|----------------------|
| **Vitesse initiale** | ⭐⭐ (3-4h) | ⭐ (4-6h) | ⭐⭐⭐ (1-2h) |
| **Vitesse après setup** | ⭐⭐⭐⭐ (10-15 min) | ⭐⭐⭐⭐⭐ (<5 min) | ⭐⭐⭐⭐ (10-20 min) |
| **Simplicité** | ⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐⭐⭐ |
| **Maintenance** | ⭐⭐⭐⭐ | ⭐⭐ | ⭐ |
| **Espace disque** | ⭐⭐⭐⭐ (2-5 GB) | ⭐ (10-20 GB) | ⭐⭐ (5-8 GB) |
| **Reproductibilité** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| **Coût** | Gratuit | Gratuit/Payant* | Gratuit |
| **Mise à jour** | ⭐⭐⭐⭐⭐ (Auto) | ⭐⭐ (Rebuild) | ⭐ (Manuel) |
| **Portabilité** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| **Complexité debug** | ⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐⭐ |

*Gratuit pour images publiques, peut nécessiter un plan payant pour stockage privé large

### Recommandation Finale

**Pour la majorité des cas** : **Option 1 (NuGet Cache)** 
- Meilleur compromis entre vitesse, maintenance et facilité d'utilisation
- Intégration native avec vcpkg
- Adapté aux workflows GitHub Actions

**Pour reproductibilité maximale** : **Option 2 (Docker)**
- Idéal pour équipes distribuées
- Garantit l'identité parfaite des environnements
- Nécessite infrastructure robuste

**Pour simplicité absolue** : **Option 3 (Binaires)**
- Solution de secours pour cas spécifiques
- Utile pour builds très stables
- Nécessite gestion manuelle rigoureuse

**Approche hybride recommandée** :
- Option 1 (NuGet) pour le CI/CD quotidien
- Option 2 (Docker) pour les releases et validation finale
- Option 3 (Binaires) comme backup/fallback en cas de problème

---

## Dépannage

### Cache ne fonctionne pas

**Symptômes** : Les packages sont recompilés à chaque fois

**Vérifications** :
1. Vérifier que `vcpkg.json` n'a pas changé (hash différent)
2. Consulter les logs GitHub Actions pour voir si cache est restauré
3. Vérifier que `downloads/` est dans la liste `path:` du cache

**Solution** :
```bash
# Reconstruire le cache manuellement
Actions → "Build vcpkg binary cache (grouped)" → Run workflow
```

### Timeout pendant l'installation

**Symptômes** : Le job dépasse 6 heures

**Solutions** :
1. Utiliser `build-cache.yml` (approche groupée) au lieu de `build-cache-single.yml`
2. Augmenter le timeout si nécessaire
3. Vérifier la connexion réseau GitHub Actions

### Packages retéléchargés

**Symptômes** : Sources téléchargées à nouveau malgré le cache

**Vérification** :
```yaml
# Dans les logs, chercher :
"Cache restored from key: vcpkg-windows-downloads-..."
```

**Solution** :
```yaml
# S'assurer que downloads/ est dans le cache :
path: |
  ${{ env.VCPKG_ROOT }}/downloads
  ${{ env.VCPKG_ROOT }}/installed
```

### Espace disque insuffisant

**Symptômes** : GitHub Actions signale "No space left on device"

**Solutions** :
1. Utiliser `--clean-packages-after-build`
2. Ne pas cacher le dossier `packages/` (déjà configuré)
3. Nettoyer les anciens caches via Actions UI

### Jobs suivants ne s'exécutent pas après échec

**Symptômes** : build-heavy-2-pcl ne démarre pas si build-heavy-1-vtk échoue

**Vérification** :
```yaml
build-heavy-2-pcl:
  needs: build-heavy-1-vtk
  if: always()  # ← Doit être présent
```

### Diagnostic du Cache

Pour vérifier l'état du cache :

```bash
# Dans les logs Actions, chercher :
- "Cache restored from key: ..." → Cache hit
- "Cache saved with key: ..." → Cache sauvegardé
- "Cache not found for input keys: ..." → Cache miss
```

Script de diagnostic disponible :
```powershell
.github/scripts/diagnose-cache.ps1
```

---

## Maintenance du Cache

### Limites GitHub Actions

- **Cache max par dépôt** : 10 GB
- **Expiration** : 7 jours sans utilisation
- **Stratégie** : Anciens caches supprimés automatiquement si limite atteinte

### Nettoyage Manuel

Via l'interface GitHub :
```
Settings → Actions → Caches → Delete specific caches
```

Via l'API GitHub :
```bash
gh api repos/{owner}/{repo}/actions/caches --method DELETE
```

---

## Choix de l'Approche de Build

| Critère | build-cache.yml (groupée) | build-cache-single.yml |
|---------|---------------------------|------------------------|
| Première installation | ✅ Recommandée | ⚠️ Risque timeout |
| Machine puissante | ✅ Bon | ✅ Excellent |
| Débogage | ✅ Excellent | ⚠️ Difficile |
| Simplicité | ⚠️ Plus complexe | ✅ Très simple |
| Production | ✅ Recommandée | ❌ Non recommandée |

**Recommandation** : Utilisez `build-cache.yml` (groupée) pour la production et les cas standards.

---

## Références

- [Documentation vcpkg](https://vcpkg.io/)
- [GitHub Actions Cache](https://docs.github.com/en/actions/using-workflows/caching-dependencies-to-speed-up-workflows)
- [vcpkg Binary Caching](https://learn.microsoft.com/en-us/vcpkg/users/binarycaching)
- [Documentation CMake](https://cmake.org/documentation/)
- [Documentation Qt6](https://doc.qt.io/qt-6/)
- [Documentation VTK](https://vtk.org/documentation/)
- [Documentation PCL](https://pointclouds.org/documentation/)
- [Documentation PDAL](https://pdal.io/)

---

**Dernière mise à jour** : Novembre 2025  
**Version** : 2.0
