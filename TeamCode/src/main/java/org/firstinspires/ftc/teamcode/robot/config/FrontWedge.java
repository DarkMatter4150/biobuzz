package org.firstinspires.ftc.teamcode.robot.config;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

/*
 * The front wedge, shared by every opmode so the servo names and positions live in one place.
 *
 * Two Axon servos (in servo mode, not continuous), one on each side, swing the wedge about
 * ROTATION_DEGREES to deploy it and back to stow it.
 *
 * INIT does NOT move it: creating a FrontWedge only sets up the servos and never sends a position,
 * so the wedge stays exactly where it is. Call holdRememberedState() once at START; that sends it
 * to whichever state the last opmode left it in (stowed or deployed), which is normally where it
 * already sits, so it doesn't move, it just starts holding.
 *
 * Memory: the state is kept in a static variable, which keeps its value when one opmode stops and
 * another starts (same idea as LeftServoMemory). It goes back to stowed when the Robot Controller
 * app restarts or the robot is powered off. So after a restart, if the wedge was left deployed,
 * it will swing to stowed at START.
 *
 * SERVO_RANGE_DEGREES is the servo's full travel, which is what positions 0 to 1 cover. On an Axon
 * that's whatever range it was set to with the Axon programmer. To check it, send 0.0 and then 1.0
 * and measure how far the horn turns. DEPLOYED_POSITION has to stay between 0 and 1.
 *
 * PWM range: Axons expect a 500-2500 microsecond signal, but the REV hub sends 600-2400 by default,
 * which cuts off the ends of their travel. We set the full 500-2500 so 0 to 1 covers the whole
 * programmed range and the angle math above comes out right.
 *
 * Direction (LEFT_DIRECTION / RIGHT_DIRECTION): set so the same position moves both sides the
 * same way. Originally only the right was reversed, but the left then deployed backwards, so now
 * both are. Changing a direction moves that servo's stowed spot to the other end of its travel,
 * so its horn has to be re-mounted afterwards (see WedgeServoTuner). Original note: the right
 * servo is mirrored from the left, so we reverse it here; then the same
 * position moves both sides the same way. Reverse in ONE place only: if a servo is also reversed
 * in the Axon programmer, the two cancel out. If one side swings the wrong way, reverse the other
 * servo here instead.
 *
 * Trim: both servos hold the same wedge, so if one is even slightly off from the other they push
 * against each other all the time (hot servos, high current, stripped gears). First line up both
 * horns at the stowed position before tightening the wedge on. If one side still sits a little
 * ahead or behind, nudge it with LEFT_TRIM_DEG or RIGHT_TRIM_DEG: a few degrees at most (they
 * are in degrees, not servo position). Change the sign if it moves the wrong way. Note a trim
 * can't push past the ends of the servo's travel: stowed is at 1.0, so a positive trim there
 * gets clipped; use a negative trim on the OTHER servo instead. Tune it with the wedge stowed
 * and deployed: the servos should be quiet (no humming or buzzing) in both spots.
 *
 * Hard stops: if a position presses the wedge into the frame, the servo pushes the whole time it
 * sits there. Set STOWED_POSITION and ROTATION_DEGREES a few degrees short of where it touches.
 *
 * Robot configuration names: "left_wedge_servo" and "right_wedge_servo".
 */
public class FrontWedge {
    public static final double ROTATION_DEGREES = 153;
    public static final double SERVO_RANGE_DEGREES = 180.0;
    // Stowed sits at the 1.0 end of the servo's travel, so deployed is ROTATION_DEGREES back from it
    public static final double STOWED_POSITION = 17.0 / SERVO_RANGE_DEGREES;
    public static final double DEPLOYED_POSITION = STOWED_POSITION + ROTATION_DEGREES / SERVO_RANGE_DEGREES;


    // Both reversed: tested on the robot, the left servo deployed the wrong way when only the
    // right was reversed (one Axon is probably already reversed in the Axon programmer).
    public static final Servo.Direction LEFT_DIRECTION = Servo.Direction.FORWARD;
    public static final Servo.Direction RIGHT_DIRECTION = Servo.Direction.FORWARD;

    // Per-side adjustment in DEGREES added to the position, so the two sides line up exactly
    public static final double LEFT_TRIM_DEG = 0.0;
    public static final double RIGHT_TRIM_DEG = 2.0;

    // Axon servos take a 500-2500 microsecond signal
    private static final PwmControl.PwmRange AXON_PWM_RANGE = new PwmControl.PwmRange(500, 2500);

    // Last commanded state. Static so it carries over from one opmode to the next.
    private static boolean deployed = false;

    private final Servo left;
    private final Servo right;

    /** Sets up the servos without moving them. */
    public FrontWedge(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, "left_wedge_servo");
        right = hardwareMap.get(Servo.class, "right_wedge_servo");
        useAxonPwmRange(left);
        useAxonPwmRange(right);
        left.setDirection(LEFT_DIRECTION);
        right.setDirection(RIGHT_DIRECTION);
    }

    /** Call once at START: holds the wedge in the state the last opmode left it in. */
    public void holdRememberedState() {
        set(deployed);
    }

    public void deploy() {
        set(true);
    }

    public void stow() {
        set(false);
    }

    public void toggle() {
        set(!deployed);
    }

    public boolean isDeployed() {
        return deployed;
    }

    private void set(boolean deploy) {
        deployed = deploy;
        double position = deploy ? DEPLOYED_POSITION : STOWED_POSITION;
        left.setPosition(Range.clip(position + LEFT_TRIM_DEG / SERVO_RANGE_DEGREES, 0, 1));
        right.setPosition(Range.clip(position + RIGHT_TRIM_DEG / SERVO_RANGE_DEGREES, 0, 1));
    }

    private static void useAxonPwmRange(Servo servo) {
        // Servos plugged into a REV hub are ServoImplEx; this check just keeps us safe if not
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(AXON_PWM_RANGE);
        }
    }
}
