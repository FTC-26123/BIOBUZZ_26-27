$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$classes = Join-Path $repoRoot 'build/pollen-model-checks'
New-Item -ItemType Directory -Force -Path $classes | Out-Null
$javaBin = if ($env:JAVA_HOME) { Join-Path $env:JAVA_HOME 'bin' } else { '' }
$javac = if ($javaBin) { Join-Path $javaBin 'javac.exe' } else { 'javac' }
$java = if ($javaBin) { Join-Path $javaBin 'java.exe' } else { 'java' }
$source = Join-Path $repoRoot 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auton/cycles'
& $javac -source 8 -target 8 -d $classes (Join-Path $source 'FeederModel.java') (Join-Path $source 'FlywheelForecast.java') (Join-Path $PSScriptRoot 'FeederModelChecks.java')
if ($LASTEXITCODE -ne 0) { throw 'Model checks did not compile' }
& $java -cp $classes FeederModelChecks
if ($LASTEXITCODE -ne 0) { throw 'Model checks failed' }
