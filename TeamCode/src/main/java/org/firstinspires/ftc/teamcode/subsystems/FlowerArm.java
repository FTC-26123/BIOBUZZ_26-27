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
    /** MASTER SWITCH. Set true only after UP_POSITION and DOWN_POSITION have been found on the robot. */
    public static final boolean ENABLED = false;
    /** Name of the servo in the robot configuration. */
    public static final String SERVO_NAME = "flowerArm";
    /** FILL IN: servo position (0..1) with the arm stowed UP. Placeholder until measured. */
    public static final double UP_POSITION = 0.10;
    /** FILL IN: servo position (0..1) with the arm DOWN for the flower bed. Placeholder until measured. */
    public static final double DOWN_POSITION = 0.60;

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
