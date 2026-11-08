param(
    # Include Pedro core/revhub 3.0.1, FTC SDK 12.0.0 and Android jars (or their Gradle equivalents).
    [Parameter(Mandatory = $true)][string]$Classpath,
    [string]$JavaHome = $env:JAVA_HOME
)
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$classes = Join-Path $repoRoot 'build/position-hold-checks'
New-Item -ItemType Directory -Force -Path $classes | Out-Null
$javac = if ($JavaHome) { Join-Path $JavaHome 'bin/javac.exe' } else { 'javac' }
$java = if ($JavaHome) { Join-Path $JavaHome 'bin/java.exe' } else { 'java' }
$source = Join-Path $repoRoot 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
& $javac -source 8 -target 8 -cp $Classpath -d $classes `
    (Join-Path $source 'pedro/Constants.java') `
    (Join-Path $source 'subsystems/TeleOpPositionHold.java') `
    (Join-Path $PSScriptRoot 'PositionHoldChecks.java')
if ($LASTEXITCODE -ne 0) { throw 'Position hold checks did not compile' }
& $java -cp "$classes;$Classpath" org.firstinspires.ftc.teamcode.subsystems.PositionHoldChecks
if ($LASTEXITCODE -ne 0) { throw 'Position hold checks failed' }
