param([string]$SimulatorRoot = "../sim-publish")
$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
Push-Location $repo
try {
    $sim = (Resolve-Path $SimulatorRoot).Path
    $java = Join-Path $sim "runtime/bin/java.exe"
    $javac = Join-Path $sim "runtime/bin/javac.exe"
    $out = Join-Path $repo "build/garden-park-checks"
    New-Item -ItemType Directory -Force $out | Out-Null
    $cp = "$out;$sim/build/classes;$sim/lib/Pedropathing.jar"
    $auton = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auton/cycles"
    & $javac -cp $cp -d $out "$sim/Controller/src/virtual_robot/biobuzz/Settings.java" `
        "$sim/Controller/src/virtual_robot/biobuzz/Physics.java" "$sim/Controller/src/virtual_robot/biobuzz/ContactMath.java" `
        "$auton/CycleRoute.java" "$auton/FrameClearance.java" "$auton/GardenParkRoute.java" `
        "$auton/CycleMechanisms.java" "$auton/FeederModel.java" "$auton/FlywheelForecast.java" `
        "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Intake.java" `
        tools/auton/GardenParkRouteChecks.java tools/auton/GardenParkSimStudy.java tools/auton/IntakeSubsystemChecks.java
    if ($LASTEXITCODE) { throw "Garden-park study compilation failed" }
    & $java -cp $cp org.firstinspires.ftc.teamcode.Auton.cycles.GardenParkRouteChecks
    if ($LASTEXITCODE) { throw "Route checks failed" }
    & $java -cp $cp IntakeSubsystemChecks
    if ($LASTEXITCODE) { throw "Intake subsystem checks failed" }
    foreach ($color in @('0','1')) {
        foreach ($profile in @(@('1.5','2.5'),@('1.3','1.6'),@('.8','.6'))) {
            & $java -cp $cp GardenParkSimStudy $color @profile
            if ($LASTEXITCODE) { throw "Physics study failed" }
        }
    }
    & $java -cp $cp org.firstinspires.ftc.teamcode.Auton.cycles.GardenParkRouteChecks --export |
        Set-Content build/garden-park-routes.tsv
} finally { Pop-Location }
