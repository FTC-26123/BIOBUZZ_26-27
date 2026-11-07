param([string]$SimulatorRoot='../sim-publish',[int]$Port=8777)
$ErrorActionPreference='Stop'
$repo=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
Push-Location $repo
$taskProcess=$null
try {
    $sim=(Resolve-Path $SimulatorRoot).Path
    $active=(Get-Content (Join-Path $sim 'build/active-build.txt') -Raw).Trim()
    $run=Join-Path $repo ('build/live-autons-'+(Get-Date -Format 'yyyyMMddHHmmss'))
    $classes=Join-Path $run 'classes'
    New-Item -ItemType Directory -Force $classes | Out-Null
    $source=Join-Path $repo 'TeamCode/src/main/java'
    $team='org/firstinspires/ftc/teamcode'
    $files=@(Get-ChildItem (Join-Path $source "$team/Auton") -Filter '*.java' -File)
    foreach($name in @('CycleRoute','FrameClearance','GardenParkRoute','GardenParkAuto','FixedShotParkAuto','ParkOnlyAuto','FeederModel','PreloadShotCounter')) {
        $files+=Get-Item (Join-Path $source "$team/Auton/cycles/$name.java")
    }
    foreach($name in @('Intake','Shooter')){$files+=Get-Item (Join-Path $source "$team/subsystems/$name.java")}
    $files+=Get-Item (Join-Path $source "$team/pedro/Constants.java")
    $compileFiles=@()
    foreach($file in $files){
        $relative=[IO.Path]::GetRelativePath($source,$file.FullName)
        $destination=Join-Path $run "ImportedTeamCode/$relative"
        New-Item -ItemType Directory -Force (Split-Path -Parent $destination) | Out-Null
        Copy-Item -LiteralPath $file.FullName -Destination $destination
        $compileFiles+=$destination
    }
    # SDK adapter ONLY: the existing simulator bot lacks the team's transfer CRServo.
    # Add that device and map its inward state to the plant gate; no ball/pose injection.
    $bot=Get-Content (Join-Path $sim 'Controller/src/virtual_robot/robots/classes/BioBuzzBot.java') -Raw
    foreach($expected in @('private double physicalGate=.5;','camera=new Limelight3A(MODEL);','gate.getPosition()-physicalGate')) {
        if(!$bot.Contains($expected)){throw 'Simulator hardware adapter changed; inspect before updating fixture'}
    }
    $bot=$bot.Replace('private double physicalGate=.5;','private double physicalGate=.5; private CRServoImpl transferServo;')
    $bot=$bot.Replace('camera=new Limelight3A(MODEL);','transferServo=new CRServoImpl(360);hardwareMap.put("transferServo",transferServo);camera=new Limelight3A(MODEL);')
    $bot=$bot.Replace('gate.getPosition()-physicalGate','(transferServo.getPower()<-.1?.27:.5)-physicalGate')
    $adapter=Join-Path $run 'adapter/virtual_robot/robots/classes/BioBuzzBot.java'
    New-Item -ItemType Directory -Force (Split-Path -Parent $adapter) | Out-Null
    Set-Content -LiteralPath $adapter -Value $bot
    $compileFiles+=$adapter
    $argsFile=Join-Path $run 'sources.args'
    $compileFiles | ForEach-Object {'"'+$_.Replace('\','/')+'"'} | Set-Content $argsFile
    & "$sim/runtime/bin/javac.exe" --module-path "$sim/vendor/javafx" --add-modules 'javafx.controls,javafx.fxml,javafx.swing,jdk.httpserver' -cp "$active;$sim/lib/*" -d $classes "@$argsFile"
    if($LASTEXITCODE){throw 'Robot/simulator compilation failed'}
    $cp="$classes;$active;$sim/Controller/src;$sim/TeamCode/src;$sim/lib/*"
    $taskProcess=Start-Process -FilePath "$sim/runtime/bin/java.exe" -WorkingDirectory $run -WindowStyle Hidden -PassThru `
        -ArgumentList @("-Dbiobuzz.port=$Port",'--module-path',('"'+$sim+'/vendor/javafx"'),'--add-modules','javafx.controls,javafx.fxml,javafx.swing,jdk.httpserver','-cp',('"'+$cp+'"'),'virtual_robot.controller.VirtualRobotApplication','--studio','--no-browser','--hidden') `
        -RedirectStandardOutput "$run/server.log" -RedirectStandardError "$run/server-error.log"
    $ready=$false
    for($attempt=0;$attempt -lt 60;$attempt++){
        try{Invoke-RestMethod "http://127.0.0.1:$Port/api/info" -TimeoutSec 1 | Out-Null;$ready=$true;break}catch{
            if($taskProcess.HasExited){throw 'Simulator stopped during startup'}
            Start-Sleep -Milliseconds 500
        }
    }
    if(!$ready){throw 'Simulator did not become ready'}
    $env:AUTON_TEST_PORT="$Port"
    foreach($name in @('RedShoot4GardenPark','BlueShoot4GardenPark','RedBasket1Shoot4Park','BlueBasket1Shoot4Park')) {
        node tools/auton/live-auton-check.cjs $name $run
        if($LASTEXITCODE){throw "Actual autonomous check failed: $name"}
    }
    Write-Output "Actual robot OpMode, Shooter/Intake and Pedro follower results: $run"
} finally {
    if($taskProcess -and !$taskProcess.HasExited){Stop-Process -InputObject $taskProcess}
    Remove-Item Env:AUTON_TEST_PORT -ErrorAction SilentlyContinue
    Pop-Location
}
