package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The servo arm that swings DOWN so that driving the robot into a flower bed lets the balls come out, and back
 * UP for normal driving.
 *
 * <p>SETUP (see NOTES/ROBOT-SETUP.md, step 7):
 * <ol>
 *   <li>Plug the servo into a servo port and name it {@link #SERVO_NAME} in the robot configuration.</li>
 *   <li>Find the two servo positions with the SDK's servo test OpMode and put them in {@link #UP_POSITION} and
 *       {@link #DOWN_POSITION}.</li>
 *   <li>Set {@link #ENABLED} to true.</li>
 * </ol>
 * Until ENABLED is true this class never moves the servo, so a wrong position cannot drive the arm into the
 * robot. A missing servo never stops an OpMode from starting.
 */
public class FlowerArm {
    /**
     * MASTER SWITCH for the real robot. Change {@code false} to {@code true} only after UP_POSITION and
     * DOWN_POSITION have been found on the robot. (In the desktop simulator the arm is always on, so the flower
     * step can be tried there first; that is what "|| InSimulator.YES" does.)
     */
    public static final boolean ENABLED = false || InSimulator.YES;
    /** Name of the servo in the robot configuration. */
    public static final String SERVO_NAME = "flowerArm";
    // The arm is a horizontal bar across the front, carried by two side bars on a 270-degree servo.
    //   position 0 = bar DOWN at the front bottom, so the robot can push it into the bottom of the flower
    //   position 1 = swung the full 270 degrees UP, resting on top of the robot, clear of the shooter
    // CHECK ON THE ROBOT (step 7.3): if the servo buzzes against a stop at either end, move that number in a little
    // (for example 0.03 and 0.97). If the arm moves the wrong way, swap the two numbers.
    /** Servo position (0..1) with the bar stowed UP on top of the robot. */
    public static final double UP_POSITION = 1.00;
    /** Servo position (0..1) with the bar DOWN at the front bottom for the flower. */
    public static final double DOWN_POSITION = 0.00;

    private Servo servo;
    private boolean down = false;
    private boolean commanded = false;
    private String problem = "";

    /** Finds the servo. Does NOT move it: the arm stays where it is until up() or down() is called. */
    public void init(HardwareMap hardwareMap) {
        servo = null;
        commanded = false;
        down = false;
        if (!ENABLED) {
            problem = "OFF (FlowerArm.ENABLED)";
            return;
        }
        try {
            servo = hardwareMap.get(Servo.class, SERVO_NAME);
            problem = servo == null ? "servo \"" + SERVO_NAME + "\" not found" : "";
        } catch (RuntimeException e) {
            servo = null;
            problem = "servo \"" + SERVO_NAME + "\" not in the robot configuration";
        }
    }

    /** @return true when the arm can actually move (switch on and servo found). */
    public boolean isAvailable() { return servo != null; }

    public void up() {
        down = false;
        if (servo != null) { servo.setPosition(UP_POSITION); commanded = true; }
    }

    public void down() {
        down = true;
        if (servo != null) { servo.setPosition(DOWN_POSITION); commanded = true; }
    }

    public void toggle() {
        if (down) up(); else down();
    }

    /** @return true if the last command was DOWN. */
    public boolean isDown() { return down; }

    public String getStatus() {
        if (servo == null) return problem;
        return !commanded ? "not moved yet" : down ? "DOWN" : "UP";
    }

    public void addTelemetry(Telemetry telemetry, String buttons) {
        if (!ENABLED) return;
        telemetry.addData("Flower arm (" + buttons + ")", getStatus());
    }
}
