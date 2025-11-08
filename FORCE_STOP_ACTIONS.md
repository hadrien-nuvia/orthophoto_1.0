# Comment Forcer l'Arrêt des Actions GitHub

Ce guide explique comment arrêter de force les workflows GitHub Actions en cours d'exécution pour ce projet.

## 🛑 Méthodes pour Arrêter les Actions

### Méthode 1: Annulation Automatique (Recommandée)

Tous les workflows de ce projet sont maintenant configurés avec des **contrôles de concurrence** qui annulent automatiquement les exécutions précédentes lorsqu'une nouvelle exécution est déclenchée.

#### Comment ça marche:

1. **Pour les workflows déclenchés par push/PR** (build.yml, build-nuget.yml):
   - Déclencher un nouveau commit ou push
   - L'exécution précédente sera automatiquement annulée

2. **Pour les workflows manuels** (build-cache.yml, build-cache-nuget.yml, verify-nuget-cache.yml, build-cache-single.yml):
   - Aller dans l'onglet **Actions**
   - Cliquer sur le workflow en cours
   - Cliquer sur **"Run workflow"** pour déclencher une nouvelle exécution
   - L'exécution précédente sera automatiquement annulée

### Méthode 2: Annulation Manuelle

Si vous préférez annuler manuellement sans déclencher une nouvelle exécution:

1. Aller dans l'onglet **Actions** du dépôt GitHub
2. Cliquer sur le workflow en cours d'exécution
3. Cliquer sur le bouton **"Cancel workflow"** (trois points ⋯ en haut à droite)

### Méthode 3: Annulation en Masse

Pour annuler toutes les exécutions en cours:

1. Aller dans l'onglet **Actions**
2. Utiliser la commande GitHub CLI:
   ```bash
   gh run list --status in_progress --json databaseId -q '.[].databaseId' | xargs -I {} gh run cancel {}
   ```

## 📋 Workflows Configurés

Tous les workflows suivants ont des contrôles de concurrence activés:

| Workflow | Groupe de Concurrence | Comportement |
|----------|----------------------|--------------|
| **Build Orthophoto App** | `build.yml-{branch/PR}` | Annule les builds précédents sur la même branche/PR |
| **Build Orthophoto App (NuGet)** | `build-nuget.yml-{branch/PR}` | Annule les builds précédents sur la même branche/PR |
| **Build vcpkg cache** | `build-cache.yml` | Annule les builds de cache précédents |
| **Build vcpkg cache (NuGet)** | `build-cache-nuget.yml` | Annule les builds de cache NuGet précédents |
| **Build vcpkg cache (single)** | `build-cache-single.yml` | Annule les builds de cache single précédents |
| **Verify NuGet Cache** | `verify-nuget-cache.yml` | Annule les vérifications précédentes |

## ⚙️ Configuration Technique

Chaque workflow contient maintenant:

```yaml
concurrency:
  group: ${{ github.workflow }}-${{ github.ref }}  # ou juste ${{ github.workflow }}
  cancel-in-progress: true
```

### Explication:

- **`group`**: Définit le groupe de concurrence (workflows identiques ou sur la même branche)
- **`cancel-in-progress: true`**: Active l'annulation automatique des exécutions précédentes du même groupe

## 🎯 Cas d'Usage Spécifiques

### Arrêter un Build qui Attend le Cache

Le workflow `build.yml` peut attendre jusqu'à 30 minutes que le cache soit construit. Pour l'arrêter:

1. **Option rapide**: Déclenchez manuellement `build.yml` avec l'option `skip_cache_build: true`
2. **Option automatique**: Faites un nouveau commit - l'ancien build sera annulé

### Arrêter une Vérification NuGet Longue

Le workflow `verify-nuget-cache.yml` peut prendre du temps. Pour l'arrêter:

1. **Option rapide**: Relancez le workflow avec `skip_package_test: true` et `skip_package_check: true`
2. **Option automatique**: Relancez simplement le workflow - l'ancien sera annulé

### Arrêter un Build de Cache Massif

Les workflows de cache (VTK, PCL, PDAL) peuvent prendre des heures:

1. Relancez simplement le workflow via l'interface GitHub Actions
2. L'exécution précédente sera annulée automatiquement

## 💡 Conseils

1. **Prévention**: Utilisez les options `skip_*` dans les workflows manuels pour sauter les étapes longues
2. **Monitoring**: Surveillez le temps d'exécution dans l'onglet Actions pour détecter les problèmes tôt
3. **Timeout**: Tous les jobs longs ont des timeouts configurés (120-360 minutes max)
4. **Logs**: Vérifiez les logs avant d'annuler pour identifier la cause des lenteurs

## 🔍 Vérification

Pour vérifier que les contrôles de concurrence fonctionnent:

1. Démarrez un workflow manuellement
2. Attendez quelques secondes
3. Démarrez-le à nouveau
4. L'exécution précédente devrait être marquée comme "Cancelled"

## 📚 Ressources

- [Documentation GitHub: Concurrency](https://docs.github.com/en/actions/using-jobs/using-concurrency)
- [Documentation GitHub: Canceling a workflow](https://docs.github.com/en/actions/managing-workflow-runs/canceling-a-workflow)
