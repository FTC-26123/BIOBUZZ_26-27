from pathlib import Path
import shutil, sys, subprocess
sim=Path(sys.argv[1]).resolve();repo=Path(sys.argv[2]).resolve();robot=repo/'TeamCode/src/main/java/org/firstinspires/ftc/teamcode';dest=sim/'ImportedTeamCode/org/firstinspires/ftc/teamcode'
expected_sim='0d6d1c5bf563799652e7aeb30ab9a9092761d233'
actual_sim=subprocess.check_output(['git','rev-parse','HEAD'],cwd=sim,text=True).strip()
if actual_sim != expected_sim: raise SystemExit('Simulator revision differs from tested '+expected_sim+'; inspect adapters before use.')
marker=sim/'.codex-live-auton-test'
if not marker.exists():
 if subprocess.check_output(['git','status','--porcelain'],cwd=sim,text=True).strip(): raise SystemExit('Use a clean, isolated simulator clone; refusing to overwrite a modified checkout.')
 marker.write_text('Owned live-auton test fixture; SDK hardware adaptation only.\n')
teleop=dest/'teleop'
if teleop.exists(): shutil.move(str(teleop),str(sim/'legacy-teleop'))
files=['Auton/TrainedFlowerAuto.java','Auton/RedTrainedTwoTipPark.java','Auton/BlueTrainedTwoTipPark.java','Auton/cycles/FrameClearance.java','subsystems/Intake.java','subsystems/Shooter.java','subsystems/FlowerArm.java','subsystems/InSimulator.java','subsystems/BulkReads.java','pedro/Constants.java']
for rel in files:
 p=dest/rel;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(robot/rel,p)
(dest/'subsystems/RobotSettings.java').write_text('package org.firstinspires.ftc.teamcode.subsystems; public final class RobotSettings {public static void ensureLoaded(){}}')
p=sim/'Controller/src/virtual_robot/robots/classes/BioBuzzBot.java';s=subprocess.check_output(['git','show','HEAD:Controller/src/virtual_robot/robots/classes/BioBuzzBot.java'],cwd=sim,text=True)
for required in ['private double physicalGate=.5;','camera=new Limelight3A(MODEL);','gate.getPosition()-physicalGate','super.updateStateAndSensors(ms);flywheel.update(ms);intake.update(ms);windmill.update(ms);']:
 if required not in s: raise SystemExit('Simulator hardware anchor changed: '+required)
s=s.replace('private double physicalGate=.5;','private double physicalGate=.5; private MechanismMotor transfer; private CRServoImpl flowerArm; private TouchSensorImpl touchUp,touchDown; private double armPosition=1;')
s=s.replace('camera=new Limelight3A(MODEL);','transfer=new MechanismMotor(false);hardwareMap.put("transfer",transfer);flowerArm=new CRServoImpl(360);touchUp=new TouchSensorImpl();touchDown=new TouchSensorImpl();touchUp.update(true);hardwareMap.put("flowerArm",flowerArm);hardwareMap.put("touchUp",touchUp);hardwareMap.put("touchDown",touchDown);camera=new Limelight3A(MODEL);')
s=s.replace('super.updateStateAndSensors(ms);flywheel.update(ms);intake.update(ms);windmill.update(ms);','super.updateStateAndSensors(ms);flywheel.update(ms);intake.update(ms);windmill.update(ms);transfer.update(ms);armPosition=Math.max(0,Math.min(1,armPosition+flowerArm.getPower()*ms*.004));touchUp.update(armPosition>=.999);touchDown.update(armPosition<=.001);')
s=s.replace('gate.getPosition()-physicalGate','(transfer.getVelocity()< -50?.27:.5)-physicalGate')
p.write_text(s)
fixture=sim/'Controller/src/virtual_robot/controller/CurrentAutonFixture.java';fixture.write_text('package virtual_robot.controller;public final class CurrentAutonFixture extends VirtualRobotApplication {public void start(javafx.stage.Stage stage)throws Exception{super.start(stage);virtual_robot.biobuzz.Lab.simulationSetup=false;Class<?> c=Class.forName("org.firstinspires.ftc.teamcode.Auton.TrainedFlowerAuto");if(System.getProperty("test.push.power")!=null)c.getField("FLOWER_PUSH_POWER").setDouble(null,Double.parseDouble(System.getProperty("test.push.power")));if(System.getProperty("test.push.seconds")!=null)c.getField("FLOWER_PUSH_SECONDS").setDouble(null,Double.parseDouble(System.getProperty("test.push.seconds")));}public static void main(String[] args){javafx.application.Application.launch(CurrentAutonFixture.class,args);}}')
print('Prepared isolated simulator:',sim)
