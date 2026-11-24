# Bootstrap the project's private interpreter and offline wheels without system Python.
[CmdletBinding()]
param(
    [ValidateSet('web','native','headless')][string]$Mode = 'web',
    [Parameter(Mandatory=$true)][string]$Venv,
    [string]$Cache = (Join-Path $env:LOCALAPPDATA 'arm-mujoco-task-sim')
)
$ErrorActionPreference = 'Stop'
if (-not [Environment]::Is64BitOperatingSystem -or
    ($env:PROCESSOR_ARCHITECTURE -ne 'AMD64' -and $env:PROCESSOR_ARCHITEW6432 -ne 'AMD64')) {
    throw 'This bundle requires x64 Windows.'
}
$packages = Join-Path $PSScriptRoot '../assets/python-win_amd64'
$record = (Get-Content -Raw -LiteralPath (Join-Path $packages 'SHA256SUMS')).Trim()
if ($record -notmatch '^([0-9a-f]{64})  ([\w.+-]+\.tar\.gz)$') { throw 'Invalid Python manifest.' }
$digest = $Matches[1]
$archive = Join-Path $packages $Matches[2]
if ((Get-FileHash -Algorithm SHA256 -LiteralPath $archive).Hash -ne $digest) { throw 'Damaged Python archive.' }
$cachePath = [IO.Path]::GetFullPath($Cache)
$target = Join-Path $cachePath ('cpython-' + $digest.Substring(0,16))
$exe = Join-Path $target 'python.exe'
if (-not (Test-Path -LiteralPath $exe -PathType Leaf)) {
    [IO.Directory]::CreateDirectory($cachePath) | Out-Null
    $stage = Join-Path $cachePath ('unpack-' + [guid]::NewGuid())
    [IO.Directory]::CreateDirectory($stage) | Out-Null
    try {
        & tar.exe -xzf $archive -C $stage
        if ($LASTEXITCODE -ne 0) { throw 'Python extraction failed.' }
        try { [IO.Directory]::Move((Join-Path $stage 'python'), $target) }
        catch { if (-not (Test-Path -LiteralPath $exe -PathType Leaf)) { throw } }
    } finally {
        if ((Split-Path -Parent ([IO.Path]::GetFullPath($stage))) -eq $cachePath) {
            Remove-Item -LiteralPath $stage -Recurse -Force
        }
    }
}
& $exe -I (Join-Path $PSScriptRoot 'environment.py') install --mode $Mode --venv $Venv
exit $LASTEXITCODE
