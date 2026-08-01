$ErrorActionPreference = "Stop"

$repoRootOutput = & git rev-parse --show-toplevel 2>$null
if ($LASTEXITCODE -ne 0) {
    throw "Run this script from inside a Git repository."
}

$repoRoot = ($repoRootOutput | Select-Object -First 1).Trim()
Set-Location -LiteralPath $repoRoot

$hookPath = ".githooks/pre-push"
if (-not (Test-Path -LiteralPath $hookPath -PathType Leaf)) {
    throw "Versioned hook not found: $hookPath"
}

$existingHooksPathOutput = & git config --local --get core.hooksPath 2>$null
if ($LASTEXITCODE -eq 0) {
    $existingHooksPath = ($existingHooksPathOutput | Select-Object -First 1).Trim()
} else {
    $existingHooksPath = ""
}

if ($existingHooksPath -and $existingHooksPath -ne ".githooks") {
    throw "core.hooksPath is already set to '$existingHooksPath'. Merge the existing hooks into .githooks before changing this setting."
}

$gitCommonDirOutput = & git rev-parse --git-common-dir
if ($LASTEXITCODE -ne 0) {
    throw "Unable to locate the Git directory."
}
$gitCommonDir = ($gitCommonDirOutput | Select-Object -First 1).Trim()
if (-not [System.IO.Path]::IsPathRooted($gitCommonDir)) {
    $gitCommonDir = Join-Path -Path $repoRoot -ChildPath $gitCommonDir
}
$legacyHook = Join-Path -Path $gitCommonDir -ChildPath "hooks/pre-push"

if (Test-Path -LiteralPath $legacyHook) {
    throw "Existing hook found at '$legacyHook'. Merge its logic into .githooks/pre-push before installing versioned hooks."
}

& git config --local core.hooksPath .githooks
if ($LASTEXITCODE -ne 0) {
    throw "Unable to configure core.hooksPath."
}

$configuredHooksPathOutput = & git config --local --get core.hooksPath
if ($LASTEXITCODE -ne 0) {
    throw "Unable to verify core.hooksPath."
}
$configuredHooksPath = ($configuredHooksPathOutput | Select-Object -First 1).Trim()
if ($configuredHooksPath -ne ".githooks") {
    throw "Git hook installation verification failed."
}

Write-Output "core.hooksPath=$configuredHooksPath"
Write-Output "pre-push hook=$hookPath"
Write-Output "Git hooks installed successfully for this repository."

