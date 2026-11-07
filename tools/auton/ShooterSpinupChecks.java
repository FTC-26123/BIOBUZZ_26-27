import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import java.lang.reflect.Proxy;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public final class ShooterSpinupChecks {
    private static int checks;
    private static void check(boolean ok,String why){checks++;if(!ok)throw new AssertionError(why);}
    public static void main(String[] args) throws Exception {
        HardwareMap hw=new HardwareMap();hw.setActive(true);double[] power={0};
        DcMotorExImpl roller=new DcMotorExImpl(MotorType.Gobilda6000,new DcMotorControllerImpl(),0);
        DcMotorExImpl windmill=new DcMotorExImpl(MotorType.Gobilda192,new DcMotorControllerImpl(),0);
        CRServoImpl transfer=new CRServoImpl(360);
        DcMotorEx motor=(DcMotorEx)Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),new Class<?>[]{DcMotorEx.class},
            (proxy,method,values)->{
                switch(method.getName()){
                    case "getVelocity":return 1500.0;
                    case "setPower":power[0]=(double)values[0];return null;
                    case "hashCode":return System.identityHashCode(proxy);
                    case "equals":return proxy==values[0];
                    case "toString":return "Recovered flywheel fixture";
                    default:return null;
                }
            });
        hw.put("intake",roller);hw.put("windmill",windmill);hw.put("transferServo",transfer);hw.put("shooter",motor);
        Intake intake=new Intake();Shooter shooter=new Shooter();intake.init(hw);shooter.init(hw,intake);
        shooter.start();Thread.sleep(70); // Let the unchanged minimum readiness pause expire.
        for(int i=0;i<20;i++){
            intake.start();intake.startWindmill();shooter.updateSpinupOnly();
            check(roller.getPower()==0&&windmill.getPower()==0&&transfer.getPower()==0,"spin-up-only overrides all intake/feed outputs");
        }
        check(power[0]>0,"flywheel PIDF runs during spin-up");
        check(shooter.getTargetLaunchPower()==1500,"same default target");
        shooter.update();
        check(roller.getPower()==Intake.INTAKE_IN&&windmill.getPower()==Intake.WINDMILL_IN&&transfer.getPower()==Intake.TRANSFER_SERVO_POWER,
            "existing update() still feeds when ready");
        shooter.updateSpinupOnly();
        check(roller.getPower()==0&&windmill.getPower()==0&&transfer.getPower()==0,"losing aim pauses every feeder output");
        shooter.stop();
        check(power[0]==0,"stop cuts flywheel output");
        System.out.println("PASS "+checks+" spin-up-only inhibition and unchanged normal Shooter update checks");
    }
}
