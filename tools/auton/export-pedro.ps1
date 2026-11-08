param([string]$SimulatorRoot = "../BIOBUZZ-sim")
# Regenerates every Pedro Pathing Visualizer project in docs/pedro from the real route code of all eight autonomous programs.
# Needs the simulator's compiled team code (load your TeamCode in the simulator first) for the SDK and route classes.
$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
Push-Location $repo
try {
    $sim = (Resolve-Path $SimulatorRoot).Path
    $active = (Get-Content (Join-Path $sim "build/active-build.txt") -Raw).Trim()
    $java = Join-Path $sim "runtime/bin/java.exe"
    $javac = Join-Path $sim "runtime/bin/javac.exe"
    $out = Join-Path $repo "build/route-export"
    New-Item -ItemType Directory -Force $out | Out-Null
    $cp = "$out;$active;$sim/lib/*"
    $auton = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auton/cycles"
    # Compile the route code from this repository (not the simulator's copy) so the export always matches the source in your working tree.
    & $javac -nowarn -cp $cp -d $out "$auton/CycleRoute.java" "$auton/FrameClearance.java" "$auton/GardenParkRoute.java" `
        "$auton/DelayedBasketTwoRoute.java" "$auton/ParkOnlyAuto.java" "$auton/PreloadShotCounter.java" `
        "$auton/AutoPaths.java" tools/auton/RouteExport.java
    if ($LASTEXITCODE) { throw "Route export classes did not compile" }
    $tsv = Join-Path $repo "build/pedro-routes.tsv"
    $rows = @()
    $rows += & $java -cp $cp org.firstinspires.ftc.teamcode.Auton.cycles.RouteExport
    # UTF-8 without a byte-order mark, so the first route name is not corrupted.
    [IO.File]::WriteAllLines($tsv, [string[]]($rows | Where-Object { $_ -match "`t" }), (New-Object Text.UTF8Encoding($false)))
    python tools/auton/export-pedro-projects.py $tsv
    Write-Output "Wrote docs/pedro/*.pp.json from $tsv"
} finally {
    Pop-Location
}
