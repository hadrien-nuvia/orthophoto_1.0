# Script de diagnostic du cache vcpkg
# Usage: Copier ce script dans un workflow GitHub Actions pour diagnostiquer les problèmes de cache

Write-Host "=== Diagnostic du Cache vcpkg ===" -ForegroundColor Green

# Vérifier les variables d'environnement
Write-Host "`n1. Variables d'environnement:" -ForegroundColor Yellow
Write-Host "VCPKG_ROOT: $env:VCPKG_ROOT"
Write-Host "VCPKG_DEFAULT_TRIPLET: $env:VCPKG_DEFAULT_TRIPLET"

# Vérifier l'existence des dossiers critiques
Write-Host "`n2. Dossiers vcpkg:" -ForegroundColor Yellow

$folders = @(
    @{Name="downloads"; Path="$env:VCPKG_ROOT\downloads"; Critical=$true},
    @{Name="installed"; Path="$env:VCPKG_ROOT\installed"; Critical=$true},
    @{Name="packages"; Path="$env:VCPKG_ROOT\packages"; Critical=$false},
    @{Name="archives"; Path="$env:VCPKG_ROOT\archives"; Critical=$false},
    @{Name="buildtrees"; Path="$env:VCPKG_ROOT\buildtrees"; Critical=$false}
)

foreach ($folder in $folders) {
    if (Test-Path $folder.Path) {
        $size = (Get-ChildItem -Path $folder.Path -Recurse -ErrorAction SilentlyContinue | 
                 Measure-Object -Property Length -Sum).Sum / 1GB
        $count = (Get-ChildItem -Path $folder.Path -ErrorAction SilentlyContinue).Count
        $status = if ($folder.Critical) {"✓ OK"} else {"OK"}
        Write-Host "  $status $($folder.Name): $([math]::Round($size, 2)) GB, $count items" -ForegroundColor Green
    } else {
        $status = if ($folder.Critical) {"✗ MANQUANT (CRITIQUE)"} else {"- Absent"}
        Write-Host "  $status $($folder.Name): N/A" -ForegroundColor $(if ($folder.Critical) {"Red"} else {"Gray"})
    }
}

# Vérifier les packages installés
Write-Host "`n3. Packages installés:" -ForegroundColor Yellow
if (Test-Path "$env:VCPKG_ROOT\vcpkg.exe") {
    & "$env:VCPKG_ROOT\vcpkg" list | Select-Object -First 20
    $totalInstalled = (& "$env:VCPKG_ROOT\vcpkg" list | Measure-Object).Count
    Write-Host "`nTotal: $totalInstalled packages installés" -ForegroundColor Cyan
} else {
    Write-Host "  vcpkg.exe non trouvé" -ForegroundColor Red
}

# Vérifier les téléchargements en cache
Write-Host "`n4. Sources téléchargées (cache downloads/):" -ForegroundColor Yellow
if (Test-Path "$env:VCPKG_ROOT\downloads") {
    $downloads = Get-ChildItem -Path "$env:VCPKG_ROOT\downloads" -ErrorAction SilentlyContinue
    if ($downloads) {
        Write-Host "  Nombre de fichiers: $($downloads.Count)" -ForegroundColor Cyan
        $topDownloads = $downloads | Sort-Object Length -Descending | Select-Object -First 5
        Write-Host "  Top 5 plus gros fichiers:" -ForegroundColor Cyan
        foreach ($file in $topDownloads) {
            $sizeMB = [math]::Round($file.Length / 1MB, 2)
            Write-Host "    - $($file.Name): $sizeMB MB"
        }
    } else {
        Write-Host "  ⚠ Aucun fichier téléchargé en cache" -ForegroundColor Yellow
    }
} else {
    Write-Host "  ✗ Dossier downloads/ manquant - Les sources seront retéléchargées!" -ForegroundColor Red
}

# Recommandations
Write-Host "`n5. Recommandations:" -ForegroundColor Yellow

$hasDownloads = Test-Path "$env:VCPKG_ROOT\downloads"
$hasInstalled = Test-Path "$env:VCPKG_ROOT\installed"

if (-not $hasDownloads) {
    Write-Host "  ✗ CRITIQUE: Ajoutez downloads/ au cache GitHub Actions!" -ForegroundColor Red
    Write-Host "    path: |" -ForegroundColor Gray
    Write-Host "      `${{ env.VCPKG_ROOT }}/downloads  # ← AJOUTER CECI" -ForegroundColor Gray
}

if (-not $hasInstalled) {
    Write-Host "  ⚠ AVERTISSEMENT: Aucun package installé trouvé" -ForegroundColor Yellow
} else {
    Write-Host "  ✓ Cache fonctionnel" -ForegroundColor Green
}

Write-Host "`n=== Fin du diagnostic ===" -ForegroundColor Green
