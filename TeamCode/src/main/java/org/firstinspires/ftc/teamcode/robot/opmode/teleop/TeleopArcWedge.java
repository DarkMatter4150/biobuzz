/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.robot.opmode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.config.FrontWedge;
import org.firstinspires.ftc.teamcode.robot.config.LeftServoMemory;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot with Mecanum
 * Wheels for the 2026-2027 FIRST® Tech Challenge. On top of a mecanum wheel drivetrain, it uses
 * one motor driving an intake roller, one servo which pulls elements out of corners, and a high-speed
 * launcher motor.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 *
 * Pedro Pathing debug telemetry: the bottom of the telemetry shows what Pedro Pathing's localizer
 * (the Pinpoint) measures, checks it against the way the driver is steering, and compares the
 * robot's top speed with the Foresight tuning in Constants. Pedro only watches here; the sticks
 * still drive the motors directly. It also runs during INIT, so you can push the robot by hand and
 * watch the pose change. Press BACK to reset the pose and the checks.
 *
 * ARC MODE (this is what's new compared with Snap Teleop Wedge)
 * There are 4 clusters of 4 AprilTags (CLUSTERS), each a flat panel. The Limelight finds the
 * center of every cluster it can see. Tapping A turns arc mode on and picks the NEAREST cluster:
 * the robot holds ARC_RADIUS_IN from that cluster's center and keeps the camera pointed at it,
 * and the left stick X moves the robot around the arc (left/right). It stays on that cluster
 * until arc mode is turned off. Nothing else changes: triggers, bumpers, Y wedge all still work.
 *
 *   A                 arc mode on / off
 *   left stick X      move along the arc
 *   dpad up / down    arc farther / closer by ARC_RADIUS_STEP_IN (stays until the opmode ends)
 *   dpad right / left run the windmill forward / backward
 *   right stick X     cancels arc mode (the driver took over turning), like it cancels a snap
 *   X / B snaps       ignored while arc mode is on
 *
 * Finding a cluster's center: with 2 or more of its tags in view (side by side), the tags give
 * both where the panel is and which way it faces, so the center is accurate even with some tags
 * hidden or the panel seen at an angle. With only 1 tag (or one column of a 2x2 grid) it guesses
 * the panel faces the camera square on, marked "(rough)" in telemetry.
 *
 * Each center is then remembered on the field using the Pinpoint, so the robot keeps circling
 * even if the tags go out of view, and is corrected every time the tags are seen again. Pressing
 * BACK resets the pose, so it also forgets the centers until the tags are seen again.
 * Without a working Pinpoint, arc mode only drives while the target cluster is in view.
 *
 * Limelight setup: AprilTag (fiducial) pipeline in slot APRILTAG_PIPELINE, with the tag family
 * and tag size set to the game's tags so the camera can measure distance (same as AprilTag
 * Approach). Measure where the camera sits on the robot and set the CAMERA_ constants.
 */

@TeleOp(name = "Arc Teleop Wedge", group = "_Compete")
//@Disabled
public class TeleopArcWedge extends OpMode {

    /*
     * Arc mode settings.
     */
    private static final String LIMELIGHT_NAME = "limelight";
    private static final int APRILTAG_PIPELINE = 0;

    /*
     * The tag clusters. Within each cluster, list the tags in the same order as
     * TAG_OFFSET_RIGHT_IN below.
     */
    private static final int[][] CLUSTERS = {
            {38, 39, 40, 41},
            {42, 43, 44, 45},
            {30, 31, 32, 33},
            {34, 35, 36, 37},
    };
    /*
     * How far each tag's center is to the RIGHT of its cluster's center, as seen standing in
     * front of the panel looking at it (inches; left is negative). Same layout for every cluster.
     * TODO: measure. Assumes a row, listed left to right, with TAG_SPACING_IN between tag centers.
     * For a 2x2 grid (top-left, top-right, bottom-left, bottom-right) use
     * {-s/2, s/2, -s/2, s/2}, where s is the distance between the two columns.
     */
    private static final double TAG_SPACING_IN = 6.0;
    private static final double[] TAG_OFFSET_RIGHT_IN = {
            -1.5 * TAG_SPACING_IN, -0.5 * TAG_SPACING_IN, 0.5 * TAG_SPACING_IN, 1.5 * TAG_SPACING_IN,
    };
    // Visible tags must spread at least this far across the panel to work out its angle
    private static final double MIN_FIT_SPREAD_IN = 3.0;

    // true when the camera is on the back of the robot looking backward
    private static final boolean CAMERA_FACES_BACKWARD = true;
    // Where the camera lens is, measured from the center of the robot: forward is positive
    // (a camera on the back has a negative value), left is positive
    private static final double CAMERA_FORWARD_OF_CENTER_IN = -8.0;
    private static final double CAMERA_LEFT_OF_CENTER_IN = 0.0;

    // Distance from the CENTER of the robot to the center of the cluster
    private static final double ARC_RADIUS_IN = 58.0;
    private static final double ARC_RADIUS_STEP_IN = 2.0;
    private static final double ARC_RADIUS_MIN_IN = 12.0;

    // Holding the distance: power per inch of error, and the most it may use
    private static final double ARC_RADIAL_KP = 0.03;
    private static final double ARC_MAX_RADIAL = 0.5;
    // Keeping the camera pointed at the arc center: power per radian of error, and the most
    private static final double ARC_TURN_KP = 0.9;
    private static final double ARC_MAX_TURN = 0.5;
    // Full left stick X is this much power along the arc
    private static final double ARC_MAX_TANGENTIAL = 0.6;
    /*
     * Turning while circling. Moving sideways around a circle needs the robot to turn at the same
     * time; this adds that turn up front so the aim doesn't lag behind. Roughly the distance from
     * the robot's center to a wheel (inches). If the camera drifts off the arc center when moving
     * along the arc, raise it if it lags behind, lower it if it turns too far.
     */
    private static final double ARC_TURN_FEEDFORWARD_IN = 8.0;

    // How much each new camera reading moves the remembered arc center (1 = trust it fully)
    private static final double CENTER_SMOOTHING = 0.3;
    private static final long MAX_STALENESS_MS = 100;
    // Without odometry, stop circling if the cluster has been out of view this long
    private static final double LOST_TIMEOUT_S = 0.3;

    private Limelight3A limelight = null;
    private String limelightError = null;

    private boolean arcMode = false;
    private boolean arcButtonWasDown = false;
    private boolean radiusUpWasDown = false;
    private boolean radiusDownWasDown = false;
    private double arcRadius = ARC_RADIUS_IN;

    private final Cluster[] clusters = new Cluster[CLUSTERS.length];
    {
        for (int i = 0; i < CLUSTERS.length; i++) clusters[i] = new Cluster(CLUSTERS[i]);
    }
    // The cluster arc mode is circling; null when arc mode is off or no cluster is known yet
    private Cluster target = null;

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private Servo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    /*
     * frontWedge: two regular servos that swing the wedge about 100 degrees to deploy it and back
     * to stow it. Tapping Y toggles between the two. The servo names and positions are in
     * robot/config/FrontWedge.java, shared with the autos.
     */
    private FrontWedge frontWedge = null;
    private boolean frontWedgeButtonWasDown = false;


    /*
     * These two variables are used to control the velocity of the launcher motor.
     * They are both in encoder ticks per second. The motors we use in the FIRST Tech Challenge
     * have encoders with a resolution of 28 ticks per revolution. We can convert this to RPM
     * by dividing the value by 28, to get to revolutions per second, before multiplying by 60
     * to get revolutions per minute.
     * We pass the target velocity variable to our motor to set the goal. We use the min velocity
     * in the launch() function to only run the windmill servo when the motor is spinning fast
     * enough to make a successful throw.
     */
    public final int LAUNCHER_TARGET_VELOCITY = 1250; //2678 RPM
    public final int LAUNCHER_MIN_VELOCITY = 1200; //2571 RPM
    public final int LAUNCHER_REVERSE_VELOCITY = 625; // half of target; reverse speed for unjamming


    /*
     * These four variables store the power we need to apply to the motors. In other cases, we may
     * choose to declare these variables inside the mecanumDrive() function, instead we declare them
     * here so that we can access them in our main loop for telemetry.
     */
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // Create a variable to set to the intake.
    double intakePower;

    /*
     * Heading snap. Tapping X turns the robot to face 90 degrees and tapping B turns it to face
     * 270 degrees, measured by the Pinpoint (counterclockwise from the way the robot faced at INIT,
     * or when BACK was last pressed). The driver can still drive and strafe during the turn.
     * Moving the right stick cancels the snap. snapTarget is null when no snap is running.
     */
    private Double snapTarget = null;
    private static final double SNAP_KP = 1.2;               // turn power per radian of error
    private static final double SNAP_MIN_POWER = 0.08;       // enough to keep turning near the end
    private static final double SNAP_MAX_POWER = 0.7;
    private static final double SNAP_TOLERANCE_DEG = 2.0;

    /*
     * Pedro Pathing debug. These stay null if Pedro can't start (for example, no Pinpoint in the
     * robot configuration); the teleop still drives, and telemetry shows why.
     */
    private Localizer localizer = null;
    private GoBildaPinpointDriver pinpoint = null;
    private String pedroError = null;
    private VoltageSensor battery = null;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // Fastest straight-line speeds seen at full stick, to compare with the Foresight tuning
    private double peakForwardSpeed;
    private double peakStrafeSpeed;

    // Results of checking that odometry agrees with the direction the driver is steering
    private String forwardCheck;
    private String strafeCheck;
    private String turnCheck;

    // Stick must be past this (and the other sticks near center) to count as a clean test move
    private static final double CHECK_STICK = 0.5;
    private static final double FULL_STICK = 0.9;
    private static final double OTHER_STICKS_CENTERED = 0.2;
    // The robot must be moving at least this fast for a direction check to count
    private static final double CHECK_SPEED_IN_PER_S = 5.0;
    private static final double CHECK_TURN_RAD_PER_S = 0.5;
    // Measured top speed this far from the Foresight value means the tuning is likely stale
    private static final double SPEED_MISMATCH_PERCENT = 15.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*
         * Start Pedro Pathing first, the same way autonomous does, so we test the real
         * configuration in Constants. Building it sets up the drive motors too, so we do it before
         * our own motor setup below; that way our directions and brake mode are the ones that stick.
         */
        try {
            Follower follower = Constants.create(hardwareMap);
            localizer = follower.localizer;
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        } catch (Exception e) {
            localizer = null;
            pedroError = e.getMessage();
        }
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            battery = sensor;
            break;
        }
        resetPedroChecks();

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step. The drive motor names come from Constants.drivetrainConfig, so they are shared
         * with Pedro Pathing and the other opmodes.
         */
        MecanumConfig drive = Constants.drivetrainConfig;
        leftFrontDrive = hardwareMap.get(DcMotor.class, drive.frontLeftName.get());
        rightFrontDrive = hardwareMap.get(DcMotor.class, drive.frontRightName.get());
        leftBackDrive = hardwareMap.get(DcMotor.class, drive.backLeftName.get());
        rightBackDrive = hardwareMap.get(DcMotor.class, drive.backRightName.get());
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(Servo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        frontWedge = new FrontWedge(hardwareMap); // doesn't move at INIT

        // The Limelight is optional: without it the teleop still drives, just no arc mode
        try {
            limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(APRILTAG_PIPELINE);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
            limelightError = e.getMessage();
        }

        /*
         * To drive forward, most robots need the motors on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. The directions also come from Constants.drivetrainConfig,
         * so if a wheel spins the wrong way, fix it there and every opmode picks it up.
         */
        leftFrontDrive.setDirection(drive.frontLeftDirection.get());
        rightFrontDrive.setDirection(drive.frontRightDirection.get());
        leftBackDrive.setDirection(drive.backLeftDirection.get());
        rightBackDrive.setDirection(drive.backRightDirection.get());

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftIntakeServo.setPosition(LeftServoMemory.position());
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        /*
         * Much like our drivetrain motors, we set the right intake servo to reverse so that it
         * works to pull elements into the intake.
         */
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        /*
         * Show Pedro Pathing telemetry before START, so you can check the Pinpoint is ready and
         * push the robot by hand to see the pose change.
         */
        telemetry.addData("Status", "Initialized");
        updateOdometry();
        readTags(); // so you can check the tags are seen before START
        addArcTelemetry();
        addPedroTelemetry(0, 0, 0);
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        frontWedge.holdRememberedState(); // stays stowed/deployed as the last opmode left it
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Update odometry and look for the tags first, so arc mode steers with this loop's
         * numbers rather than last loop's.
         */
        updateOdometry();
        readTags();
        arcButtons();

        /*
         * Here we call a function called mecanumDrive. The mecanumDrive function takes the input from
         * the joysticks, and applies power to the drive motors to move the robot as requested
         * by the driver. Moving the left joystick forwards/back moves all motors forwards/back,
         * moving the right joystick left/right rotates the robot clockwise or counterclockwise,
         * and moving the left joystick left moves the motors in the right way to create a sideways
         * "strafe" movement. Combinations of these inputs can be used to create more complex maneuvers.
         * Note, moving the joystick forward on most gamepads results in a negative signal, so
         * we invert it before passing it to the function.
         */
        if (target != null) {
            arcDrive(target, gamepad1.left_stick_x);
        } else {
            // Arc mode on but no cluster seen yet: drive normally until one is
            mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, snapRotate(gamepad1.right_stick_x));
        }

        /*
         * Set the intake power variable to equal the right trigger, minus the left trigger.
         * Each trigger outputs a signal from 0-1, with 0 as fully released, and 1 fully depressed.
         * This gives us proportional control of the intake speed. The speed increases as we pull
         * the right trigger further. It's occasionally helpful to be able to reverse the intake,
         * so we also factor in the left trigger. If the left trigger is fully depressed,
         * the intakePower variable will be -1. If the right trigger is fully depressed, the variable
         * will be 1. If the driver pulls both triggers, the intake will remain off.
         * We use this technique (creating a variable, and setting it to our control inputs) to
         * allow us to avoid setting the same motors/servos power more than once per loop. That can
         * create erratic behavior.
         */
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        launch();

        /*
         * Here we set our intake motor and servos to their intake power. The order of operations
         * here is important though. The gamepad triggers define the starting point for the intake
         * power variable in each loop of our code, but inside our launch function we also sometimes
         * change the intake power. So we need to give our launch function a chance to modify the
         * variable before we write it to our motor and servos.
         */

        intake.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        frontWedge();

        /*
         * Show motor powers on the Driver Station via telemetry.
         */
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Triggers", "left (%.2f), right (%.2f)", gamepad1.left_trigger, gamepad1.right_trigger);
        telemetry.addData("Snap (X=90, B=270)", snapTarget == null
                ? "off" : String.format("turning to %.0f deg", Math.toDegrees(snapTarget)));
        telemetry.addData("Front wedge (Y)", frontWedge.isDeployed() ? "DEPLOYED" : "stowed");

        addArcTelemetry();
        addPedroTelemetry(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }

    /*
     * Pedro's localizer used to update inside addPedroTelemetry; it now updates here, at the top
     * of each loop, because arc mode needs the pose before it drives.
     */
    void updateOdometry() {
        if (localizer == null) return;
        if (gamepad1.back) {
            localizer.setPose(Pose.zero());
            resetPedroChecks();
            // The remembered cluster centers were in the old coordinates
            for (Cluster cluster : clusters) cluster.known = false;
            target = null;
        }
        localizer.update();
    }

    /*
     * A toggles arc mode; dpad up/down change the radius. Moving the right stick cancels arc
     * mode, the same way it cancels a heading snap.
     */
    void arcButtons() {
        boolean aDown = gamepad1.a;
        if (aDown && !arcButtonWasDown) {
            arcMode = !arcMode;
        }
        arcButtonWasDown = aDown;
        if (Math.abs(gamepad1.right_stick_x) > OTHER_STICKS_CENTERED) {
            arcMode = false;
        }

        if (arcMode) {
            snapTarget = null;
            /*
             * Pick the cluster once, when arc mode starts (or as soon as one has been seen), and
             * stay on it until arc mode is turned off, so the robot doesn't jump between clusters
             * while it's circling. To choose a cluster another way, change pickTarget().
             */
            if (target == null || !usable(target)) target = pickTarget();
        } else {
            target = null;
        }

        if (gamepad1.dpad_up && !radiusUpWasDown) {
            arcRadius += ARC_RADIUS_STEP_IN;
        }
        if (gamepad1.dpad_down && !radiusDownWasDown) {
            arcRadius = Math.max(ARC_RADIUS_MIN_IN, arcRadius - ARC_RADIUS_STEP_IN);
        }
        radiusUpWasDown = gamepad1.dpad_up;
        radiusDownWasDown = gamepad1.dpad_down;
    }

    /*
     * For now: the nearest cluster whose center we know. null if none is known yet.
     */
    Cluster pickTarget() {
        Cluster nearest = null;
        for (Cluster cluster : clusters) {
            if (!usable(cluster)) continue;
            if (nearest == null || cluster.distance() < nearest.distance()) nearest = cluster;
        }
        return nearest;
    }

    /*
     * Looks for every cluster's tags, works out each seen cluster's center, and remembers it.
     * Then, for every cluster we know, works out where its center is relative to the robot right
     * now (robotX/robotY).
     */
    void readTags() {
        for (Cluster cluster : clusters) cluster.tagsSeen = 0;

        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()
                    && result.getStaleness() < MAX_STALENESS_MS
                    && result.getPipelineIndex() == APRILTAG_PIPELINE) {
                double dir = CAMERA_FACES_BACKWARD ? -1 : 1;
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    // Camera space: x right, y down, z straight out of the lens
                    if (tag.getTargetPoseCameraSpace() == null) continue;
                    Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
                    for (Cluster cluster : clusters) {
                        int slot = cluster.slotOf(tag.getFiducialId());
                        if (slot < 0) continue;
                        // Robot space: x forward, y left. A backward camera's right is the robot's left.
                        cluster.seenX[cluster.tagsSeen] = CAMERA_FORWARD_OF_CENTER_IN + dir * p.z;
                        cluster.seenY[cluster.tagsSeen] = CAMERA_LEFT_OF_CENTER_IN - dir * p.x;
                        cluster.seenOffset[cluster.tagsSeen] = TAG_OFFSET_RIGHT_IN[slot];
                        cluster.tagsSeen++;
                    }
                }
            }
        }

        for (Cluster cluster : clusters) {
            if (cluster.tagsSeen > 0) updateCenter(cluster);
            if (localizer != null && cluster.known) {
                // Field to robot: subtract the robot's position, then undo its heading
                Pose pose = localizer.pose();
                double cos = Math.cos(pose.heading());
                double sin = Math.sin(pose.heading());
                double dx = cluster.fieldX - pose.x();
                double dy = cluster.fieldY - pose.y();
                cluster.robotX = dx * cos + dy * sin;
                cluster.robotY = -dx * sin + dy * cos;
            }
        }
    }

    /*
     * Works out a cluster's center from the tags of it seen this loop. Every tag sits on the panel
     * at  center + offset * panelRight,  where offset is its TAG_OFFSET_RIGHT_IN and panelRight is
     * the direction along the panel (to the right, as seen facing it).
     *
     *   Tags at 2+ different offsets   fit a line through them: that gives panelRight, and the
     *                                  center even when some tags are hidden or the panel is
     *                                  seen at an angle. This is the accurate one.
     *   Otherwise (one tag, or one     assume the panel faces the camera square on. Only used
     *   column of a grid)              when we don't already have a better remembered center.
     */
    void updateCenter(Cluster c) {
        int n = c.tagsSeen;
        double meanX = 0, meanY = 0, meanOffset = 0;
        double minOffset = Double.MAX_VALUE, maxOffset = -Double.MAX_VALUE;
        for (int i = 0; i < n; i++) {
            meanX += c.seenX[i] / n;
            meanY += c.seenY[i] / n;
            meanOffset += c.seenOffset[i] / n;
            minOffset = Math.min(minOffset, c.seenOffset[i]);
            maxOffset = Math.max(maxOffset, c.seenOffset[i]);
        }

        double rightX, rightY;
        boolean fitted = false;
        if (maxOffset - minOffset >= MIN_FIT_SPREAD_IN) {
            // Least-squares direction of the line the tags lie on
            double sumX = 0, sumY = 0;
            for (int i = 0; i < n; i++) {
                double d = c.seenOffset[i] - meanOffset;
                sumX += d * (c.seenX[i] - meanX);
                sumY += d * (c.seenY[i] - meanY);
            }
            double length = Math.hypot(sumX, sumY);
            rightX = sumX / length;
            rightY = sumY / length;
            fitted = length > 0;
        } else {
            rightX = 0;
            rightY = 0;
        }
        if (!fitted) {
            // A rough guess is still better than nothing, but don't let it spoil a good one
            if (c.known && localizer != null) return;
            // Square to the camera: panel right is the camera's right, across the line of sight
            double sightX = meanX - CAMERA_FORWARD_OF_CENTER_IN;
            double sightY = meanY - CAMERA_LEFT_OF_CENTER_IN;
            double length = Math.hypot(sightX, sightY);
            if (length < 1) return;
            rightX = sightY / length;
            rightY = -sightX / length;
        }
        c.fitted = fitted;

        double robotX = meanX - meanOffset * rightX;
        double robotY = meanY - meanOffset * rightY;

        if (localizer != null) {
            // Remember it on the field so it survives the tags going out of view
            Pose pose = localizer.pose();
            double cos = Math.cos(pose.heading());
            double sin = Math.sin(pose.heading());
            double fieldX = pose.x() + robotX * cos - robotY * sin;
            double fieldY = pose.y() + robotX * sin + robotY * cos;
            if (c.known) {
                c.fieldX += CENTER_SMOOTHING * (fieldX - c.fieldX);
                c.fieldY += CENTER_SMOOTHING * (fieldY - c.fieldY);
            } else {
                c.fieldX = fieldX;
                c.fieldY = fieldY;
            }
        } else {
            c.robotX = robotX;
            c.robotY = robotY;
        }
        c.known = true;
        c.sinceSeen.reset();
    }

    /*
     * True when we know where a cluster's center is well enough to drive around it.
     */
    boolean usable(Cluster c) {
        if (!c.known) return false;
        return localizer != null || c.sinceSeen.seconds() < LOST_TIMEOUT_S;
    }

    /*
     * Drives around the target cluster's center. Three parts are added together:
     *   radial      toward / away from the center to hold arcRadius
     *   tangential  the driver's stick, sideways around the circle (stick right = robot's right)
     *   turn        keeps the camera pointed at the center, plus the turn circling needs
     * Radial and tangential are worked out relative to where the center actually is, so they
     * still go the right way while the aim is catching up.
     */
    void arcDrive(Cluster c, double stick) {
        double r = c.distance();
        if (r < 1) {
            mecanumDrive(0, 0, 0); // sitting on the center; no direction makes sense
            return;
        }
        double dir = CAMERA_FACES_BACKWARD ? -1 : 1;
        // Unit vector from the robot toward the center (x forward, y left)
        double ux = c.robotX / r;
        double uy = c.robotY / r;
        // Around the circle: the robot's right when the camera is pointed at the center
        double tx = dir * uy;
        double ty = -dir * ux;

        double radial = Range.clip(ARC_RADIAL_KP * (r - arcRadius), -ARC_MAX_RADIAL, ARC_MAX_RADIAL);
        double tangential = stick * ARC_MAX_TANGENTIAL;
        double forward = ux * radial + tx * tangential;
        double left = uy * radial + ty * tangential;

        // mecanumDrive turns clockwise for positive, so the sign is flipped
        double turn = Range.clip(-ARC_TURN_KP * aimError(c), -ARC_MAX_TURN, ARC_MAX_TURN);
        turn += -dir * tangential * ARC_TURN_FEEDFORWARD_IN / r;

        mecanumDrive(forward, -left, turn); // mecanumDrive's strafe is positive to the right
    }

    void addArcTelemetry() {
        telemetry.addLine();
        telemetry.addLine("--- Arc mode (A) ---");
        if (limelight == null) {
            telemetry.addData("Limelight", "NOT FOUND: %s", limelightError);
        }
        String state;
        if (!arcMode) {
            state = "off";
        } else if (target == null) {
            state = "ON, waiting to see a cluster (driving normally)";
        } else {
            state = "ON, circling " + target.name + (target.tagsSeen > 0 ? "" : " (out of view: odometry)");
        }
        telemetry.addData("Arc", state);
        telemetry.addData("Radius (dpad)", "target %.0f in", arcRadius);
        for (Cluster c : clusters) {
            String line;
            if (!c.known) {
                line = "not seen yet";
            } else {
                line = String.format("%.0f in away, %.0f deg off aim, %d tag%s in view%s",
                        c.distance(), Math.toDegrees(aimError(c)), c.tagsSeen,
                        c.tagsSeen == 1 ? "" : "s",
                        c.tagsSeen > 0 && !c.fitted ? " (rough)" : "");
            }
            telemetry.addData((c == target ? "> " : "  ") + c.name, line);
        }
    }

    /*
     * How far (radians) a cluster's center is from where the camera points: front camera wants
     * it straight ahead, back camera straight behind. Positive means the center is
     * counterclockwise of the camera's aim. Wrapped into -180..180 degrees.
     */
    double aimError(Cluster c) {
        double error = Math.atan2(c.robotY, c.robotX) - (CAMERA_FACES_BACKWARD ? Math.PI : 0);
        return Math.atan2(Math.sin(error), Math.cos(error));
    }

    /*
     * One panel of 4 tags, and what we know about where its center is.
     */
    private static class Cluster {
        final String name;
        final int[] ids; // in TAG_OFFSET_RIGHT_IN order

        boolean known = false;
        boolean fitted = false;        // last center came from 2+ tags, not the rough guess
        double fieldX, fieldY;         // remembered center, Pedro field coordinates
        double robotX, robotY;         // center relative to the robot this loop (x fwd, y left)
        final ElapsedTime sinceSeen = new ElapsedTime();

        // Tags of this cluster seen this loop, robot space
        int tagsSeen;
        final double[] seenX, seenY, seenOffset;

        Cluster(int[] ids) {
            this.ids = ids;
            this.name = ids[0] + "-" + ids[ids.length - 1];
            seenX = new double[ids.length];
            seenY = new double[ids.length];
            seenOffset = new double[ids.length];
        }

        /** Position of the tag in ids (and TAG_OFFSET_RIGHT_IN), or -1 if not in this cluster. */
        int slotOf(int id) {
            for (int i = 0; i < ids.length; i++) if (ids[i] == id) return i;
            return -1;
        }

        double distance() {
            return Math.hypot(robotX, robotY);
        }
    }

    /*
     * Returns the turn power to use this loop. Normally that's just the driver's right stick, but
     * while a snap is running we turn toward snapTarget instead. Like mecanumDrive, positive
     * rotate is clockwise, while Pedro's heading is counterclockwise, so the sign is flipped.
     */
    double snapRotate(double stickRotate) {
        if (localizer == null) {
            return stickRotate; // no odometry, so no snap
        }
        if (gamepad1.x) {
            snapTarget = Math.toRadians(90);
        } else if (gamepad1.b) {
            snapTarget = Math.toRadians(270);
        }
        if (Math.abs(stickRotate) > OTHER_STICKS_CENTERED) {
            snapTarget = null; // the driver took over
        }
        if (snapTarget == null) {
            return stickRotate;
        }

        // Shortest way around: wrap the error into -180..180 degrees
        double error = snapTarget - localizer.pose().heading();
        error = Math.atan2(Math.sin(error), Math.cos(error));
        if (Math.abs(error) < Math.toRadians(SNAP_TOLERANCE_DEG)) {
            snapTarget = null;
            return 0;
        }

        double power = SNAP_KP * Math.abs(error);
        power = Math.max(SNAP_MIN_POWER, Math.min(SNAP_MAX_POWER, power));
        return -Math.signum(error) * power;
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        /*
         * Send calculated power to wheels
         */
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /*
     * Pedro Pathing debug telemetry. forward, strafe and rotate are what the driver is asking for
     * (same as mecanumDrive: forward, right and clockwise are positive), so we can check that the
     * odometry agrees with the way the robot is being driven.
     *
     * Pedro measures the robot with x forward, y to the LEFT, and heading counterclockwise, so
     * strafing right should read a negative y speed and turning right a negative turn rate.
     */
    void addPedroTelemetry(double forward, double strafe, double rotate) {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        telemetry.addLine();
        telemetry.addLine("--- Pedro Pathing (BACK resets) ---");
        if (localizer == null) {
            telemetry.addData("Pedro", "NOT RUNNING: %s", pedroError);
            return;
        }

        // localizer.update() and the BACK reset now happen in updateOdometry()
        Pose pose = localizer.pose();
        Twist twist = localizer.twist();

        /*
         * READY is good. CALIBRATING should only last a moment after INIT; keep the robot still.
         * Any FAULT status means a pod is unplugged or the Pinpoint isn't wired right.
         */
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());

        /*
         * Push the robot a measured distance (for example 48 inches along a tile seam) and check
         * x or y changes by the same amount. If not, the pod type in Constants is wrong.
         */
        telemetry.addData("Pose", "x %.1f in, y %.1f in, heading %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Speed", "fwd %.1f, left %.1f in/s, turn %.0f deg/s",
                twist.vx, twist.vy, Math.toDegrees(twist.omega));

        /*
         * Direction checks. These assume the robot physically moves the way the stick says; if it
         * doesn't, fix the motor directions first (StarterBot Hardware Test helps).
         */
        boolean onlyForward = Math.abs(forward) > CHECK_STICK
                && Math.abs(strafe) < OTHER_STICKS_CENTERED && Math.abs(rotate) < OTHER_STICKS_CENTERED;
        boolean onlyStrafe = Math.abs(strafe) > CHECK_STICK
                && Math.abs(forward) < OTHER_STICKS_CENTERED && Math.abs(rotate) < OTHER_STICKS_CENTERED;
        boolean onlyTurn = Math.abs(rotate) > CHECK_STICK
                && Math.abs(forward) < OTHER_STICKS_CENTERED && Math.abs(strafe) < OTHER_STICKS_CENTERED;

        if (onlyForward && Math.abs(twist.vx) > CHECK_SPEED_IN_PER_S) {
            forwardCheck = Math.signum(twist.vx) == Math.signum(forward)
                    ? "OK"
                    : "WRONG: odometry says the other way. Flip xPodDirection in Constants";
        }
        if (onlyStrafe && Math.abs(twist.vy) > CHECK_SPEED_IN_PER_S) {
            strafeCheck = Math.signum(twist.vy) == -Math.signum(strafe)
                    ? "OK"
                    : "WRONG: odometry says the other way. Flip yPodDirection in Constants";
        }
        if (onlyTurn && Math.abs(twist.omega) > CHECK_TURN_RAD_PER_S) {
            turnCheck = Math.signum(twist.omega) == -Math.signum(rotate)
                    ? "OK"
                    : "WRONG: heading turns the other way. Is the Pinpoint mounted upside down?";
        }
        telemetry.addData("Forward check", forwardCheck);
        telemetry.addData("Strafe check", strafeCheck);
        telemetry.addData("Turn check", turnCheck);

        /*
         * Foresight check. The Foresight tuner measured the robot's top speed at full power. If
         * the robot is now much faster or slower at full stick (new motors, wheels, gearing,
         * weight, or motor directions changed since tuning), Foresight's numbers are stale.
         */
        if (Math.abs(forward) > FULL_STICK && Math.abs(strafe) < OTHER_STICKS_CENTERED
                && Math.abs(rotate) < OTHER_STICKS_CENTERED) {
            peakForwardSpeed = Math.max(peakForwardSpeed, Math.abs(twist.vx));
        }
        if (Math.abs(strafe) > FULL_STICK && Math.abs(forward) < OTHER_STICKS_CENTERED
                && Math.abs(rotate) < OTHER_STICKS_CENTERED) {
            peakStrafeSpeed = Math.max(peakStrafeSpeed, Math.abs(twist.vy));
        }

        ForesightConfig foresight = Constants.foresightConfig;
        telemetry.addLine("--- Foresight ---");
        telemetry.addData("Top fwd speed", compareSpeed(peakForwardSpeed,
                foresight.maxAchievableForwardVelocity.get(), "drive forward at full stick"));
        telemetry.addData("Top strafe speed", compareSpeed(peakStrafeSpeed,
                foresight.maxAchievableStrafeVelocity.get(), "strafe at full stick"));
        telemetry.addData("Natural decel", "fwd %.1f, strafe %.1f in/s^2",
                foresight.naturalForwardDeceleration.get(), foresight.naturalStrafeDeceleration.get());

        // Low battery makes the robot slower than when it was tuned; a slow loop makes paths jerky
        if (battery != null) telemetry.addData("Battery", "%.2f V", battery.getVoltage());
        telemetry.addData("Loop time", "%.0f ms", loopMs);
    }

    /*
     * Describes the fastest speed seen next to the speed Foresight was tuned with.
     */
    private static String compareSpeed(double measured, double tuned, String howToTest) {
        if (measured < 1) {
            return String.format("tuned %.1f in/s; %s to test", tuned, howToTest);
        }
        double percent = (measured - tuned) / tuned * 100;
        String verdict;
        if (percent > SPEED_MISMATCH_PERCENT) {
            verdict = "FASTER than tuned: re-run ForesightTuner";
        } else if (percent < -SPEED_MISMATCH_PERCENT) {
            verdict = "slower: try a longer run; if it stays low check battery or re-tune";
        } else {
            verdict = "OK";
        }
        return String.format("%.1f vs tuned %.1f in/s (%+.0f%%) %s", measured, tuned, percent, verdict);
    }

    private void resetPedroChecks() {
        peakForwardSpeed = 0;
        peakStrafeSpeed = 0;
        forwardCheck = "not tested: push left stick forward";
        strafeCheck = "not tested: push left stick right";
        turnCheck = "not tested: push right stick right";
    }

    /*
     * Tapping Y deploys the front wedge; tapping Y again stows it. We only act on the moment the
     * button goes down (not every loop while it's held), otherwise holding Y would flip the wedge
     * back and forth every loop.
     */
    void frontWedge() {
        boolean buttonDown = gamepad1.y;
        if (buttonDown && !frontWedgeButtonWasDown) {
            frontWedge.toggle();
        }
        frontWedgeButtonWasDown = buttonDown;
    }

    void launch() {
        /*
         * Holding the left bumper runs everything backwards to clear a jam: the launcher spins in
         * reverse, the windmill pushes elements back out of the launcher, and the intake pushes
         * them out of the robot. This takes priority over the right bumper, so we return early
         * and skip the normal launch logic below.
         */
        if (gamepad1.left_bumper) {
            launcher.setVelocity(-LAUNCHER_REVERSE_VELOCITY);
            windmillServo.setPower(-1);
            intakePower = -1;
            return;
        }

        /*
         * Calling gamepad1.right_bumper returns a boolean which will be true if the bumper is
         * held down, and false if it is not. Notably, this will continue to be true for every
         * cycle of our code that the driver holds down that bumper.
         * The first step of our launch() function is checking to see if the user is currently
         * holding down the right gamepad. If they are, then we want to start spinning up the launcher.
         * Otherwise, we start spinning the launcher down.
         */
        if (gamepad1.right_bumper) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        /*
         * Here we ask if the driver is currently pressing the right bumper, AND the launcher is
         * spinning fast enough to make a successful shot. If it is, then we will turn on the
         * windmill servo to start feeding the elements into the launcher motor. We also
         * add some power to the intake power. This can sometimes help dislodge stuck elements from
         * inside the hopper.
         */
        if (gamepad1.right_bumper && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillServo.setPower(1);
            intakePower += 0.5;
        } else {
            // Dpad right runs the windmill forward, dpad left runs it backward
            if (gamepad1.dpad_right) {
                windmillServo.setPower(1);
            } else if (gamepad1.dpad_left) {
                windmillServo.setPower(-1);
            } else {
                windmillServo.setPower(0);
            }
        }
    }
}