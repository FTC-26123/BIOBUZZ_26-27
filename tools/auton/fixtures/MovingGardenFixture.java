package virtual_robot.controller;

/** Keep the team's drive tuning; disable the lab's optional demo setup injection. */
public final class MovingGardenFixture extends VirtualRobotApplication {
    @Override public void start(javafx.stage.Stage stage)throws Exception{
        super.start(stage);
        virtual_robot.biobuzz.Lab.simulationSetup=false;
    }
    public static void main(String[] args){javafx.application.Application.launch(MovingGardenFixture.class,args);}
}
