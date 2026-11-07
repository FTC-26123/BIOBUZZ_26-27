// Run from robot-fixes using tools/auton/run-garden-park-sim.ps1.
// Unmodified simulator Physics, fresh standard field; no extra balls or teammate injection.
// Bounded trapezoid drive, NOT execution of the FTC OpMode or the dyn4j follower.
// Fixture truth gates its feeder; robot code uses the existing estimated release counter.
import virtual_robot.biobuzz.Physics;
import virtual_robot.biobuzz.Settings;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.Auton.cycles.*;

public final class GardenParkSimStudy {
    private static final double DT=.005;
    private final Physics plant;
    private final GardenParkRoute route;
    private final int alliance;
    private final double speed,accel,sign;
    private boolean collecting,guards;
    private int geometrySamples;

    private String fallback="none";
    private static final class ParkNeeded extends RuntimeException {}

    GardenParkSimStudy(int alliance,double speed,double accel){
        Settings settings=new Settings();settings.alliance=alliance;
        plant=new Physics(settings);route=new GardenParkRoute(alliance==1);
        this.alliance=alliance;this.speed=speed;this.accel=accel;sign=alliance==0?1:-1;
        plant.robotX=route.initial.x()*.0254;plant.robotY=route.initial.y()*.0254;
        plant.heading=route.initial.heading()-Math.PI/2;
    }
    private Pose pose(){return new Pose(plant.robotX/.0254,plant.robotY/.0254,plant.heading+Math.PI/2);}
    private static double duration(double distance,double velocity,double acceleration){
        return distance<velocity*velocity/acceleration?2*Math.sqrt(distance/acceleration)
            :distance/velocity+velocity/acceleration;
    }
    private static double fraction(double t,double total,double distance,double velocity,double acceleration){
        if(distance<1e-9)return 1;
        double base=duration(distance,velocity,acceleration),local=t/(total/base);
        double peak=Math.min(velocity,Math.sqrt(distance*acceleration)),ramp=peak/acceleration,cruise=base-2*ramp;
        double traveled=local<ramp?.5*acceleration*local*local:local<ramp+cruise
            ?.5*acceleration*ramp*ramp+peak*(local-ramp)
            :distance-.5*acceleration*Math.pow(Math.max(0,base-local),2);
        return Math.min(1,traveled/distance);
    }
    private void step(double intake,double shooter,double gate){
        plant.step(DT,intake,shooter,gate);
        double h=plant.heading+Math.PI/2,half=.2286*(Math.abs(Math.cos(h))+Math.abs(Math.sin(h)));
        require(Math.abs(plant.robotX)+half<=Physics.HALF+.00001
            &&Math.abs(plant.robotY)+half<=Physics.HALF+.00001,"rotated 18-inch wall clearance");
        require(FrameClearance.clear(plant.robotX,plant.robotY),"frame keepout");
        require(Math.abs(plant.robotX+1.55*sign)>half+.2286
            ||Math.abs(plant.robotY-1.19*sign)>half+.2286,"reserved teammate footprint");
        geometrySamples++;
        if(guards&&plant.time>=29.5-route.parkReserve(pose())){
            fallback="parking reserve";throw new ParkNeeded();
        }
    }
    private void pause(double seconds){
        for(int k=0;k<Math.ceil(seconds/DT);k++){
            plant.robotVx=plant.robotVy=plant.robotOmega=0;step(collecting?Intake.INTAKE_POWER:0,0,1);
        }
    }
    private void drive(Pose destination,boolean pickup){
        collecting=pickup;
        for(Pose next:FrameClearance.path(pose(),destination,alliance==1)){
            double x0=plant.robotX,y0=plant.robotY,h0=plant.heading;
            double dx=next.x()*.0254-x0,dy=next.y()*.0254-y0;
            double turn=Math.atan2(Math.sin(next.heading()-Math.PI/2-h0),Math.cos(next.heading()-Math.PI/2-h0));
            double distance=Math.hypot(dx,dy),velocity=pickup?GardenParkRoute.SWEEP_METERS_PER_SECOND:speed;
            double total=Math.max(duration(distance,velocity,accel),duration(Math.abs(turn),3,6));
            int steps=Math.max(1,(int)Math.ceil(total/DT));total=steps*DT;
            for(int k=1;k<=steps;k++){
                double u=fraction(k*DT,total,distance,velocity,accel);
                double w=fraction(k*DT,total,Math.abs(turn),3,6);
                double x=x0+dx*u,y=y0+dy*u,h=h0+turn*w;
                plant.robotVx=(x-plant.robotX)/DT;plant.robotVy=(y-plant.robotY)/DT;plant.robotOmega=(h-plant.heading)/DT;
                plant.robotX=x;plant.robotY=y;plant.heading=h;step(pickup?Intake.INTAKE_POWER:0,0,1);
            }
            pause(.10); // Explicit settling; stops at each planned waypoint.
        }
    }
    private void fire(){
        collecting=false;
        pause(.4); // Estimated spin-up only: actual Shooter PIDF and flywheel recovery are not run here.
        int before=plant.shots;double until=plant.time+3;
        while(plant.shots<before+4&&plant.time<until)step(Intake.INTAKE_POWER,1500,.27);
        if(plant.shots!=before+4){fallback="incomplete reload";throw new ParkNeeded();}
        pause(.1);
    }
    private void run(){
        guards=true;
        try{
            for(CycleRoute.Stage stage:route.stages){
                drive(stage.pose(),stage.action()==CycleRoute.Action.SWEEP);
                if(stage.action()==CycleRoute.Action.SWEEP)pause(GardenParkRoute.CAPTURE_DWELL_SECONDS);
                if(stage.action()==CycleRoute.Action.FIRE)fire();
            }
        }catch(ParkNeeded expected){/* Same policy as robot: give up scoring and reserve the return. */}
        guards=false;collecting=false;
        for(Pose next:route.parkingPath(pose()))drive(next,false);
        double parked=plant.time;
        while(plant.time<30)pause(DT); // Ball flight and damped tipping continue while robot is parked.
        int garden=0,oppositeGarden=0,flower=0,nectar=0;
        for(Physics.Ball ball:plant.balls)if(ball.collected){
            if(ball.origin.equals("garden")){garden++;if(ball.spawnAlliance!=alliance)oppositeGarden++;}
            if(ball.origin.equals("flower"))flower++;
            if(ball.color>=0)nectar++;
        }
        System.out.printf("%s v=%.2f a=%.2f parked=%.3fs shots=%d hits=%d tips=%d held=%d ownGarden=%d oppositeGarden=%d flowers=%d nectar=%d fallback=%s samples=%d%n",
            alliance==0?"RED":"BLUE",speed,accel,parked,plant.shots,plant.hits,plant.hives[alliance].tips,
            plant.inventory.size(),garden-oppositeGarden,oppositeGarden,flower,nectar,fallback,geometrySamples);
        require(parked<=29.7,"park before cutoff");
        require(flower==0&&nectar==0&&oppositeGarden==0,"only our garden, pollen only, no flowers");
        require(plant.capacityViolations==0,"four-ball capacity");
        require(plant.shots==4&&plant.hives[alliance].tips==1,"only the opening basket batch");
        require(garden==4&&plant.inventory.size()==4,"park with four own-garden pollen for TeleOp");
    }
    private static void require(boolean ok,String why){if(!ok)throw new AssertionError(why);}
    public static void main(String[] args){
        int color=args.length>0?Integer.parseInt(args[0]):0;
        double velocity=args.length>1?Double.parseDouble(args[1]):1.5;
        double acceleration=args.length>2?Double.parseDouble(args[2]):2.5;
        new GardenParkSimStudy(color,velocity,acceleration).run();
    }
}
