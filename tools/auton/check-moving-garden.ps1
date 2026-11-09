param([string]$PedroCoreJar)
$ErrorActionPreference='Stop'
$repo=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
if(!$PedroCoreJar){
    $cached=Join-Path $env:USERPROFILE '.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.1'
    $PedroCoreJar=(Get-ChildItem $cached -Recurse -Filter 'core-3.0.1.jar' | Select-Object -First 1).FullName
}
if(!$PedroCoreJar -or !(Test-Path -LiteralPath $PedroCoreJar)){throw 'Pass -PedroCoreJar with the Pedro 3.0.1 core JAR'}
$classes=Join-Path $repo 'build/moving-garden-checks'
New-Item -ItemType Directory -Force $classes | Out-Null
$source=Join-Path $repo 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auton/cycles'
$javac=if($env:JAVA_HOME){Join-Path $env:JAVA_HOME 'bin/javac.exe'}else{'javac'}
$java=if($env:JAVA_HOME){Join-Path $env:JAVA_HOME 'bin/java.exe'}else{'java'}
$files=@('CycleRoute','FrameClearance','GardenParkRoute','GardenStrafe','MovingAim','MovingShotSolver','TransitPath') |
    ForEach-Object {Join-Path $source "$_.java"}
& $javac -source 8 -target 8 -cp $PedroCoreJar -d $classes @files (Join-Path $PSScriptRoot 'MovingGardenChecks.java')
if($LASTEXITCODE){throw 'Moving garden checks did not compile'}
& $java -cp "$classes;$PedroCoreJar" org.firstinspires.ftc.teamcode.Auton.cycles.MovingGardenChecks
if($LASTEXITCODE){throw 'Moving garden checks failed'}
