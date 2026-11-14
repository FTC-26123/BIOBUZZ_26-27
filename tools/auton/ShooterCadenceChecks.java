package org.firstinspires.ftc.teamcode.subsystems;
import java.lang.reflect.*;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Scripted speed samples exercise the real shooter's count/recovery/feed gate. */
public class ShooterCadenceChecks {
    private static int checks;
    private static void check(boolean value,String message){checks++;if(!value)throw new AssertionError(message);}
    private static void set(Shooter shooter,String name,Object value)throws Exception{
        Field f=Shooter.class.getDeclaredField(name);f.setAccessible(true);f.set(shooter,value);
    }
    private static void sample(Shooter shooter,double now,double speed)throws Exception{
        ElapsedTime.now=now;set(shooter,"velocity",speed);
        Method m=Shooter.class.getDeclaredMethod("handleAutoShooting");m.setAccessible(true);m.invoke(shooter);
    }
    private static Shooter fixture(boolean override)throws Exception{
        ElapsedTime.now=0;Shooter s=new Shooter();set(s,"intake",new Intake());
        s.setAutoTarget(1450);s.resetTrim();if(override)s.setMinimumBetweenBallsSeconds(.25);s.start();
        sample(s,.06,1480);sample(s,.50,1480);return s;
    }
    private static boolean feeding(Shooter s){return "FEED".equals(s.getFeedStage());}
    public static void main(String[] args)throws Exception{
        Shooter.BETWEEN_BALLS_SEC=.1;
        Shooter s=fixture(true);check(feeding(s),"first ball may feed at speed without extra recovery delay");
        sample(s,.60,1430); // 50-tick drop still inside the target's speed window.
        check(s.getShotCount()==1&&!feeding(s),"detected shot must close feed even if speed stays in window");
        sample(s,.66,1450);check(!feeding(s),"old window timer must not bypass the new shot's delay");
        sample(s,.849,1450);check(!feeding(s),"feed must remain closed before .25 seconds");
        sample(s,.851,1450);check(feeding(s),"feed may reopen after .25 seconds continuously in window");
        sample(s,.86,1480);sample(s,1.0,1380);
        check(s.getShotCount()==2&&!feeding(s),"out-of-window shot must also pause feed");
        sample(s,1.1,1450);sample(s,1.349,1450);
        check(!feeding(s),"speed recovery must restart the full settle wait");
        sample(s,1.351,1450);check(feeding(s),"recovered speed may reopen after the settle wait");
        check(s.getTargetLaunchPower()==1450,"cadence changes must not change the 1450 target");
        s=fixture(false);sample(s,.60,1430);sample(s,.71,1450);
        check(feeding(s),"another shooter instance must retain its .1-second default");
        for(double invalid:new double[]{-1,Double.NaN,Double.POSITIVE_INFINITY}){
            boolean rejected=false;try{s.setMinimumBetweenBallsSeconds(invalid);}catch(IllegalArgumentException expected){rejected=true;}
            check(rejected,"invalid recovery wait must be rejected");
        }
        System.out.println("PASS: "+checks+" real-shooter recovery/cadence checks, including counted drops inside the speed window.");
    }
}
