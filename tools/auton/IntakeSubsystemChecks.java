import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import java.lang.reflect.Proxy;
import org.firstinspires.ftc.teamcode.Auton.cycles.CycleMechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/** Exercise the legacy autonomous helper against the existing Intake and SDK motor stand-ins. */
public final class IntakeSubsystemChecks {
    private static int checks;
    private static void check(boolean ok,String why){checks++;if(!ok)throw new AssertionError(why);}
    public static void main(String[] args){
        HardwareMap hw=new HardwareMap();hw.setActive(true);
        DcMotorExImpl roller=new DcMotorExImpl(MotorType.Gobilda6000,new DcMotorControllerImpl(),0);
        DcMotorExImpl windmill=new DcMotorExImpl(MotorType.Gobilda192,new DcMotorControllerImpl(),0);
        DcMotorExImpl transfer=new DcMotorExImpl(MotorType.Gobilda6000,new DcMotorControllerImpl(),0);
        // A recovered flywheel allows the test to exercise the feed permission transition.
        DcMotorEx flywheel=(DcMotorEx)Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),new Class<?>[]{DcMotorEx.class},
            (proxy,method,values)->{
                if(method.getName().equals("getVelocity"))return CycleMechanisms.SPEED;
                if(method.getName().equals("hashCode"))return System.identityHashCode(proxy);
                if(method.getName().equals("equals"))return proxy==values[0];
                if(method.getName().equals("toString"))return "Recovered flywheel fixture";
                return null;
            });
        hw.put("intake",roller);hw.put("windmill",windmill);hw.put("transfer",transfer);hw.put("shooter",flywheel);
        CycleMechanisms auto=new CycleMechanisms(hw);
        check(roller.getDirection()==DcMotorSimple.Direction.REVERSE,"subsystem roller direction");
        check(windmill.getDirection()==DcMotorSimple.Direction.FORWARD,"subsystem windmill direction");
        check(roller.getMode()==DcMotor.RunMode.RUN_WITHOUT_ENCODER,"subsystem roller mode");
        check(windmill.getMode()==DcMotor.RunMode.RUN_WITHOUT_ENCODER,"windmill is open loop (no PID)");
        check(windmill.getZeroPowerBehavior()==DcMotor.ZeroPowerBehavior.FLOAT,"windmill has no brake");
        check(transfer.getMode()==DcMotor.RunMode.RUN_USING_ENCODER,"subsystem transfer velocity mode");
        check(transfer.getZeroPowerBehavior()==DcMotor.ZeroPowerBehavior.BRAKE,"transfer brakes at zero power from init");
        check(transfer.getPower()==0,"transfer starts stopped");
        check(transfer.getDirection()==Intake.TRANSFER_DIRECTION,"transfer direction constant applied");
        auto.collect(true);
        check(roller.getPower()==Intake.INTAKE_IN,"uses current subsystem intake power");
        check(transfer.getPower()<0,"uses subsystem inward transfer velocity");
        check(windmill.getPower()<0&&windmill.getPower()>-0.2,"collection turns the windmill slowly in reverse");
        auto.feed(0,true);auto.feed(.13,true);
        check(windmill.getPower()>0,"permitted feeding starts the windmill forward");
        auto.close();
        check(transfer.getZeroPowerBehavior()==DcMotor.ZeroPowerBehavior.BRAKE,"closing leaves the transfer braking");
        check(roller.getPower()==0&&windmill.getPower()==0&&transfer.getPower()==0,"closing stops every intake actuator");
        auto.collect(true);auto.stop();
        check(transfer.getZeroPowerBehavior()==DcMotor.ZeroPowerBehavior.BRAKE,"stop leaves the transfer braking");
        check(roller.getPower()==0&&windmill.getPower()==0&&transfer.getPower()==0,"stop cancels collection");
        System.out.println("PASS "+checks+" existing Intake initialization, collection, feeder and shutdown checks");
    }
}
