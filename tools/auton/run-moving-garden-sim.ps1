param([string]$SimulatorRoot='../sim-publish',[int]$Port=8781,[ValidateRange(1,20)][int]$Repeats=2)
$ErrorActionPreference='Stop'
$repo=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$sim=(Resolve-Path $SimulatorRoot).Path
$active=(Get-Content (Join-Path $sim 'build/active-build.txt') -Raw).Trim()
$run=Join-Path $repo ('build/moving-garden-sim-'+(Get-Date -Format 'yyyyMMddHHmmss'))
$classes=Join-Path $run 'classes'
New-Item -ItemType Directory -Force $classes | Out-Null
$source=Join-Path $repo 'TeamCode/src/main/java'
$team='org/firstinspires/ftc/teamcode'
$files=@(Get-ChildItem (Join-Path $source "$team/Auton") -Filter '*.java' -File)
$files+=Get-ChildItem (Join-Path $source "$team/Auton/cycles") -Filter '*.java' -File
foreach($name in @('Intake','Shooter')){$files+=Get-Item (Join-Path $source "$team/subsystems/$name.java")}
$files+=Get-Item (Join-Path $source "$team/pedro/Constants.java")
$compileFiles=@()
foreach($file in $files){
    $relative=$file.FullName.Substring((Resolve-Path $source).Path.Length+1) # Windows PowerShell 5.1 has no GetRelativePath
    $destination=Join-Path $run "ImportedTeamCode/$relative"
    New-Item -ItemType Directory -Force (Split-Path -Parent $destination) | Out-Null
    Copy-Item -LiteralPath $file.FullName -Destination $destination
    $compileFiles+=$destination
}
$bot=Get-Content (Join-Path $sim 'Controller/src/virtual_robot/robots/classes/BioBuzzBot.java') -Raw
# The simulator needs a DC motor named "transfer" (the team Intake's passive-lock transfer); the adapter below adds it.
if(!$bot.Contains('hardwareMap.put("transfer",')){
foreach($expected in @('private double physicalGate=.5;','camera=new Limelight3A(MODEL);')){
        if(!$bot.Contains($expected)){throw 'Simulator hardware changed; review transfer adapter'}
    }
    # Supply missing team hardware; do not change Physics, drive dynamics or camera detection.
    $bot=$bot.Replace('private double physicalGate=.5;','private double physicalGate=.5; private DcMotorExImpl transferMotor;')
    $bot=$bot.Replace('camera=new Limelight3A(MODEL);','transferMotor=new DcMotorExImpl(MotorType.Gobilda6000,motorController1,3);hardwareMap.put("transfer",transferMotor);camera=new Limelight3A(MODEL);')
    # The team Intake drives a DC motor named "transfer" ; running inward (power below -0.1) opens the gate.
    if($bot.Contains('transferServo.getPower()')){$bot=$bot.Replace('transferServo.getPower()','transferMotor.getPower()')}
    else{$bot=$bot.Replace('gate.getPosition()-physicalGate','(transferMotor.getPower()<-.1?.27:.5)-physicalGate')}
    # Surface scheduled-physics exceptions instead of reporting a frozen robot as a controller fault.
    $bot=$bot.Replace('public synchronized void updateStateAndSensors(double ms){','public synchronized void updateStateAndSensors(double ms){ try {')
    $bot=$bot.Replace('camera.sample();webcam.sample();}','camera.sample();webcam.sample();} } catch(RuntimeException|Error fault){fault.printStackTrace();throw fault;}')
    $adapter=Join-Path $run 'adapter/virtual_robot/robots/classes/BioBuzzBot.java'
    New-Item -ItemType Directory -Force (Split-Path -Parent $adapter) | Out-Null
    Set-Content -LiteralPath $adapter -Value $bot
    $compileFiles+=$adapter
}
$compileFiles+=Join-Path $PSScriptRoot 'fixtures/MovingGardenFixture.java'
$tagFolder=Join-Path $run 'web/assets/field'
New-Item -ItemType Directory -Force $tagFolder | Out-Null
Get-ChildItem (Join-Path $sim 'web/assets/field') -Filter 'apriltag-board-*.png' | Copy-Item -Destination $tagFolder
$argsFile=Join-Path $run 'sources.args'
$compileFiles | ForEach-Object {'"'+$_.Replace('\','/')+'"'} | Set-Content $argsFile
& "$sim/runtime/bin/javac.exe" --module-path "$sim/vendor/javafx" --add-modules 'javafx.controls,javafx.fxml,javafx.swing,jdk.httpserver' -cp "$active;$sim/lib/*" -d $classes "@$argsFile"
if($LASTEXITCODE){throw 'Robot/simulator compilation failed'}
$cp="$classes;$active;$sim/Controller/src;$sim/TeamCode/src;$sim/lib/*"
$simProcess=$null
$oldPort=$env:AUTON_TEST_PORT;$oldResults=$env:AUTON_RESULTS
try{
    $portProbe=[System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Loopback,$Port)
    try{$portProbe.Start()}finally{$portProbe.Stop()}
    $simProcess=Start-Process -FilePath "$sim/runtime/bin/java.exe" -WorkingDirectory $run -WindowStyle Hidden -PassThru `
        -ArgumentList @("-Dbiobuzz.port=$Port",'-Djavafx.cachedir=fx-cache','--module-path',('"'+$sim+'/vendor/javafx"'),'--add-modules','javafx.controls,javafx.fxml,javafx.swing,jdk.httpserver','-cp',('"'+$cp+'"'),'virtual_robot.controller.MovingGardenFixture','--studio','--no-browser','--hidden') `
        -RedirectStandardOutput "$run/server.log" -RedirectStandardError "$run/server-error.log"
    $ready=$false
    for($attempt=0;$attempt -lt 40;$attempt++){
        try{Invoke-RestMethod "http://127.0.0.1:$Port/api/info" -TimeoutSec 1 | Out-Null;$ready=$true;break}
        catch{if($simProcess.HasExited){throw 'Simulator failed; see server-error.log'};Start-Sleep -Milliseconds 300}
    }
    if(!$ready){throw 'Simulator server did not become ready'}
    $env:AUTON_TEST_PORT="$Port";$failed=$false
    for($trial=1;$trial -le $Repeats;$trial++){
        $env:AUTON_RESULTS=Join-Path $run "trial-$trial"
        node (Join-Path $PSScriptRoot 'check-moving-garden-sim.cjs') RedGardenMovingShots BlueGardenMovingShots
        if($LASTEXITCODE){$failed=$true}
    }
    Write-Output "Actual OpMode/motor/ball results: $run"
    if($failed){throw 'At least one full-cycle scoring check failed; inspect all trial results'}
}finally{
    $env:AUTON_TEST_PORT=$oldPort;$env:AUTON_RESULTS=$oldResults
    if($simProcess -and !$simProcess.HasExited){Stop-Process -InputObject $simProcess}
}
