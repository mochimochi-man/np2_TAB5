param(
    [string]$Output = 'PC98N.ROM'
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$tempOutput = $null
Push-Location $PSScriptRoot
try {
    if (-not (Get-Command nasm -ErrorAction SilentlyContinue)) {
        throw 'nasm was not found in PATH'
    }
    if (-not (Get-Command python -ErrorAction SilentlyContinue)) {
        throw 'python was not found in PATH'
    }

    $tempOutput = $Output + '.tmp'
    & nasm -Wall -Werror -f bin -o $tempOutput pc98n.asm
    if ($LASTEXITCODE -ne 0) { throw 'nasm failed' }

    & python verify_rom.py $tempOutput --fix-checksum --output $Output --variants
    if ($LASTEXITCODE -ne 0) { throw 'ROM verification failed' }

    & nasm -Wall -Werror -f bin -o BIOSCHK.COM bioschk.asm
    if ($LASTEXITCODE -ne 0) { throw 'BIOSCHK.COM build failed' }
    Write-Host 'OK: BIOSCHK.COM (DOS BIOS identification utility)'
}
finally {
    if ($tempOutput -and (Test-Path -LiteralPath $tempOutput)) {
        Remove-Item -LiteralPath $tempOutput -Force
    }
    Pop-Location
}
