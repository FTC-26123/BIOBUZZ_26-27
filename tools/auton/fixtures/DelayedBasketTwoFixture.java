package virtual_robot.controller;

import javafx.animation.AnimationTimer;
import javafx.stage.Stage;
import virtual_robot.biobuzz.Physics;
import virtual_robot.biobuzz.Settings;
import virtual_robot.biobuzz.StudioServer;
import virtual_robot.robots.classes.BioBuzzBot;

/** Test-only teammate delivery, not robot autonomous code.
 * Three reserved partner preloads fall into Basket 1 at ~4 s; the normal plant swings the hive.
 * Our robot's four preloads, drive, feeder, shots and shot counting are not injected or overridden. */
public final class DelayedBasketTwoFixture extends VirtualRobotApplication {
    @Override public void start(Stage stage) throws Exception {
        super.start(stage);
        new AnimationTimer(){
            int generation=-1,delivered;
            public void handle(long now){
                var controller=getControllerHandle();String program=controller.studioSelected();
                if(!program.endsWith("Basket2Wait15Shoot4Park")||!controller.studioPhase().equals("running"))return;
                Physics p=BioBuzzBot.MODEL;
                synchronized(p){
                    if(generation!=p.resetGeneration){generation=p.resetGeneration;delivered=0;}
                    if(delivered>=3||p.time<4+.42*delivered)return;
                    Physics.Hive hive=p.hives[program.contains(".Blue")?1:0];
                    if(hive.tips!=0||hive.swinging)return;
                    Physics.Ball ball=new Physics.Ball(-1,p.cfg);
                    ball.origin="partner-preload";ball.spawnAlliance=hive.color;
                    ball.x=hive.x+.09*(delivered-1);ball.y=hive.y()+hive.side*.075;
                    ball.z=hive.z()+.15;ball.vy=-hive.side*.15;ball.vz=-3;ball.state="flight";
                    ball.entryCounted=true; // Exclude fixture deliveries from our robot's hit counter only.
                    p.balls.add(ball);delivered++;
                }
            }
        }.start();
    }
    public static void main(String[] args){Settings.S.load();StudioServer.loadProfile();launch(args);}
}
