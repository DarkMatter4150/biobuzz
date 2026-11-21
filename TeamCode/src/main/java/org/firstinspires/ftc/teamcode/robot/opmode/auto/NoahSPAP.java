package org.firstinspires.ftc.teamcode.robot.opmode.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.config.Modifier;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.config.FrontWedge;
import org.firstinspires.ftc.teamcode.robot.config.LeftServoMemory;

/*
 * Autonomous for the mecanum StarterBot, using the same hardware and launch logic as
 * BioBuzzStarterbotTeleopMecanum, and the same Limelight setup as TeleopArcWedge:
 *
 *   1. LAUNCH: spin the launcher up for LAUNCH_SECONDS. Once it is at speed, the windmill
 *      feeds elements into it. Only the windmill runs; the intake and intake servos stay off.
 *   2. Deploy the wedge and drive straight to intakePrep (13, 46.6, 180).
 *   3. Run the intake and drive slowly (PICKUP_SPEED_SCALE of top speed) to pickup (10, 46.6, 180),
 *      then pause there PICKUP_PAUSE_SECONDS with the intake still running.
 *   4. Back up to intakePrep (13, 46.6, 180), then stop the intake.
 *   5. Stow the wedge and drive to shoot (48.36, 130.72, 95).
 *   6. TARGET: use the Limelight to find the nearest AprilTag cluster and turn in place so the
 *      camera points at its center. The launcher spins up at the same time. Moves on once the
 *      aim is within AIM_TOLERANCE_DEG, or after AIM_TIMEOUT_SECONDS (then it launches at
 *      whatever heading it has, 95 if no tags were seen).
 *   7. LAUNCH again for LAUNCH_SECONDS, still aiming.
 *   8. Stop aiming and drive to park (13.71, 91, 180) through parkVia (30, 91, 180), then hold
 *      that spot until the end of autonomous.
 *
 * Pedro Pathing uses the Pinpoint from Constants to know where the robot is.
 */
@Autonomous(name = "NoahSPAP", group = "Compete")
public class NoahSPAP extends LinearOpMode {

    // Keep these in sync with BioBuzzStarterbotTeleopMecanum
    private static final int LAUNCHER_TARGET_VELOCITY = 1250; // 2678 RPM
    private static final int LAUNCHER_MIN_VELOCITY = 1200;    // 2571 RPM
    private static final double FEED_INTAKE_POWER = 0.5;
    private static final double PICKUP_INTAKE_POWER = 1.0;

    private static final double LAUNCH_SECONDS = 7.0;

    // Step 3 drives at this fraction of top speed so the intake has time to grab elements
    private static final double PICKUP_SPEED_SCALE = 0.2;
    // Then waits this long at the pickup spot with the intake still running
    private static final double PICKUP_PAUSE_SECONDS = 2.0;

    /*
     * Limelight targeting. Keep these in sync with TeleopArcWedge.
     */
    private static final String LIMELIGHT_NAME = "limelight";
    private static final int APRILTAG_PIPELINE = 0;
    private static final int[][] CLUSTERS = {
            {38, 39, 40, 41},
            {42, 43, 44, 45},
            {30, 31, 32, 33},
            {34, 35, 36, 37},
    };
    // How far each tag's center is to the RIGHT of its cluster's center (inches), in CLUSTERS order
    private static final double TAG_SPACING_IN = 6.0;
    private static final double[] TAG_OFFSET_RIGHT_IN = {
            -1.5 * TAG_SPACING_IN, -0.5 * TAG_SPACING_IN, 0.5 * TAG_SPACING_IN, 1.5 * TAG_SPACING_IN,
    };
    private static final double MIN_FIT_SPREAD_IN = 3.0;
    private static final boolean CAMERA_FACES_BACKWARD = true;
    private static final double CAMERA_FORWARD_OF_CENTER_IN = -8.0;
    private static final double CAMERA_LEFT_OF_CENTER_IN = 0.0;
    private static final long MAX_STALENESS_MS = 100;
    private static final double CENTER_SMOOTHING = 0.3;

    // Step 6: aimed is close enough when within this, and give up waiting after this long
    private static final double AIM_TOLERANCE_DEG = 2.0;
    private static final double AIM_TIMEOUT_SECONDS = 2.0;
    // Only send Pedro a new heading when the aim moves at least this much, so it doesn't jitter
    private static final double AIM_UPDATE_DEG = 0.5;

    private DcMotor intake;
    private DcMotorEx launcher;
    private Servo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;
    private Limelight3A limelight = null;
    private String limelightError = null;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.0, 9.0, 270);
    private final Pose intakePrep = poseFactory.of(13.0, 46.6, 180);
    private final Pose pickup = poseFactory.of(10.0, 46.6, 180);
    private final Pose shoot = poseFactory.of(48.36, 130.72, 95);
    private final Pose parkVia = poseFactory.of(30.0, 91.0, 180);
    private final Pose park = poseFactory.of(13.71, 91.0, 180);

    // Targeting state: the cluster we're aiming at (-1 = none yet) and its center on the field
    private int targetCluster = -1;
    private boolean centerKnown = false;
    private double centerFieldX, centerFieldY;
    private int tagsSeen = 0;
    private double heldAimHeading = Double.NaN;

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(start);

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(Servo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        FrontWedge frontWedge = new FrontWedge(hardwareMap); // doesn't move at INIT

        // The Limelight is optional: without it the auto still runs, it just launches at 95 degrees
        try {
            limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(APRILTAG_PIPELINE);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
            limelightError = e.getMessage();
        }

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo.setPosition(LeftServoMemory.position());
        setFeed(0);

        if (limelight == null) telemetry.addData("Limelight", "NOT FOUND: %s", limelightError);
        telemetry.update();
        waitForStart();
        frontWedge.holdRememberedState(); // stays stowed/deployed as the last opmode left it

        // 1. LAUNCH. The robot stays still; follower.update() keeps the position up to date.
        launch(follower, "LAUNCH 1", false);

        // 2. Deploy the wedge and drive straight to intakePrep
        frontWedge.deploy();
        follower.follow(Paths.line(start, intakePrep).linear(start, intakePrep));
        followPath(follower, "TO INTAKE PREP (wedge deploying)");

        // 3. Intake on, slowly in to the pickup spot
        setIntake(PICKUP_INTAKE_POWER);
        // maxPathSpeed is a fraction of top speed (0-1), not a speed in in/s
        Modifier slow = Constants.foresightConfig.maxPathSpeed.at(PICKUP_SPEED_SCALE);
        follower.follow(Paths.line(intakePrep, pickup).linear(intakePrep, pickup).with(slow));
        followPath(follower, "PICKUP (intake running, slow)");

        // Pause at the pickup spot, still holding position and running the intake
        ElapsedTime pauseTimer = new ElapsedTime();
        while (opModeIsActive() && pauseTimer.seconds() < PICKUP_PAUSE_SECONDS) {
            follower.update();

            telemetry.addData("Step", "PICKUP PAUSE (%.1f s left)", PICKUP_PAUSE_SECONDS - pauseTimer.seconds());
            addPoseTelemetry(follower);
            telemetry.update();
        }

        // 4. Back up to intakePrep (intake still running), then stop the intake
        follower.follow(Paths.line(pickup, intakePrep).linear(pickup, intakePrep));
        followPath(follower, "BACK UP (intake running)");
        setIntake(0);

        // 5. Stow the wedge and drive to the shooting spot
        frontWedge.stow();
        follower.follow(Paths.line(intakePrep, shoot).linear(intakePrep, shoot));
        followPath(follower, "TO SHOOT (wedge stowing)");

        // 6. TARGET: aim at the nearest tag cluster while the launcher spins up
        ElapsedTime aimTimer = new ElapsedTime();
        while (opModeIsActive() && aimTimer.seconds() < AIM_TIMEOUT_SECONDS) {
            follower.update();
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            double aimError = aim(follower);
            if (!Double.isNaN(aimError) && Math.abs(aimError) < AIM_TOLERANCE_DEG) break;

            telemetry.addData("Step", "TARGET (%.1f s left)", AIM_TIMEOUT_SECONDS - aimTimer.seconds());
            addAimTelemetry(aimError);
            addPoseTelemetry(follower);
            telemetry.update();
        }

        // 7. LAUNCH again, still aiming
        launch(follower, "LAUNCH 2", true);

        // 8. Stop aiming and park through parkVia, then hold there until the end
        Pose here = follower.pose();
        follower.follow(Paths.through(here, parkVia, park).linear(here, park));

        while (opModeIsActive()) {
            follower.update();

            telemetry.addData("Step", follower.following() ? "PARK" : "DONE (holding position)");
            if (follower.following()) {
                telemetry.addData("Distance left (in)", "%.1f", follower.distanceToEndpoint());
            }
            addPoseTelemetry(follower);
            telemetry.update();
        }

        follower.stop();
        launcher.setVelocity(0);
        setIntake(0);
        if (limelight != null) limelight.stop();
    }

    /*
     * Spins the launcher for LAUNCH_SECONDS, running the windmill (only) once it is at speed.
     * With aiming on, it keeps turning toward the target cluster the whole time.
     */
    private void launch(Follower follower, String step, boolean aiming) {
        ElapsedTime launchTimer = new ElapsedTime();
        while (opModeIsActive() && launchTimer.seconds() < LAUNCH_SECONDS) {
            follower.update();
            double aimError = aiming ? aim(follower) : Double.NaN;

            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            boolean atSpeed = launcher.getVelocity() > LAUNCHER_MIN_VELOCITY;
            windmillServo.setPower(atSpeed ? 1 : 0); // windmill only; the intake stays off while launching

            telemetry.addData("Step", "%s (%.1f s left)", step, LAUNCH_SECONDS - launchTimer.seconds());
            telemetry.addData("Launcher", "%.0f / %d ticks/s %s", launcher.getVelocity(),
                    LAUNCHER_TARGET_VELOCITY, atSpeed ? "(feeding)" : "(spinning up)");
            if (aiming) addAimTelemetry(aimError);
            addPoseTelemetry(follower);
            telemetry.update();
        }
        launcher.setVelocity(0);
        setFeed(0);
    }

    /*
     * Reads the tags, updates the target cluster's center, and tells Pedro to hold the shooting
     * spot facing so the camera points at that center. Returns the aim error in degrees, or NaN
     * if no cluster has been seen yet (Pedro then just keeps holding where it is).
     */
    private double aim(Follower follower) {
        readTags(follower);
        if (!centerKnown) return Double.NaN;

        // Field to robot: subtract the robot's position, then undo its heading
        Pose pose = follower.pose();
        double cos = Math.cos(pose.heading());
        double sin = Math.sin(pose.heading());
        double dx = centerFieldX - pose.x();
        double dy = centerFieldY - pose.y();
        double robotX = dx * cos + dy * sin;
        double robotY = -dx * sin + dy * cos;

        // Same as TeleopArcWedge.aimError: positive = center is counterclockwise of the camera's aim
        double error = Math.atan2(robotY, robotX) - (CAMERA_FACES_BACKWARD ? Math.PI : 0);
        error = Math.atan2(Math.sin(error), Math.cos(error));

        double targetHeading = pose.heading() + error;
        if (Double.isNaN(heldAimHeading)
                || Math.abs(angleDiff(targetHeading, heldAimHeading)) > Math.toRadians(AIM_UPDATE_DEG)) {
            heldAimHeading = targetHeading;
            follower.hold(shoot.withHeading(targetHeading));
        }
        return Math.toDegrees(error);
    }

    /*
     * Finds the tags of every cluster in view. The first time any cluster is seen, the nearest
     * one becomes the target; after that we stay on it. Its center is worked out the same way as
     * TeleopArcWedge.updateCenter and remembered on the field.
     */
    private void readTags(Follower follower) {
        tagsSeen = 0;
        if (limelight == null) return;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()
                || result.getStaleness() >= MAX_STALENESS_MS
                || result.getPipelineIndex() != APRILTAG_PIPELINE) {
            return;
        }

        int n = CLUSTERS[0].length;
        double[][] seenX = new double[CLUSTERS.length][n];
        double[][] seenY = new double[CLUSTERS.length][n];
        double[][] seenOffset = new double[CLUSTERS.length][n];
        int[] count = new int[CLUSTERS.length];

        double dir = CAMERA_FACES_BACKWARD ? -1 : 1;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            // Camera space: x right, y down, z straight out of the lens
            if (tag.getTargetPoseCameraSpace() == null) continue;
            Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
            for (int c = 0; c < CLUSTERS.length; c++) {
                int slot = slotOf(CLUSTERS[c], tag.getFiducialId());
                if (slot < 0 || count[c] >= n) continue;
                // Robot space: x forward, y left. A backward camera's right is the robot's left.
                seenX[c][count[c]] = CAMERA_FORWARD_OF_CENTER_IN + dir * p.z;
                seenY[c][count[c]] = CAMERA_LEFT_OF_CENTER_IN - dir * p.x;
                seenOffset[c][count[c]] = TAG_OFFSET_RIGHT_IN[slot];
                count[c]++;
            }
        }

        // Pick the target the first time: the nearest cluster in view
        if (targetCluster < 0) {
            double nearest = Double.MAX_VALUE;
            for (int c = 0; c < CLUSTERS.length; c++) {
                if (count[c] == 0) continue;
                double d = Math.hypot(seenX[c][0], seenY[c][0]);
                if (d < nearest) {
                    nearest = d;
                    targetCluster = c;
                }
            }
            if (targetCluster < 0) return;
        }

        int c = targetCluster;
        tagsSeen = count[c];
        if (tagsSeen == 0) return;
        updateCenter(follower, seenX[c], seenY[c], seenOffset[c], tagsSeen);
    }

    /*
     * Center of the target cluster from the tags seen this loop (see TeleopArcWedge.updateCenter).
     * With tags at 2+ different offsets it fits the panel's direction; otherwise it assumes the
     * panel faces the camera square on, but only if there isn't a better center remembered.
     */
    private void updateCenter(Follower follower, double[] x, double[] y, double[] offset, int n) {
        double meanX = 0, meanY = 0, meanOffset = 0;
        double minOffset = Double.MAX_VALUE, maxOffset = -Double.MAX_VALUE;
        for (int i = 0; i < n; i++) {
            meanX += x[i] / n;
            meanY += y[i] / n;
            meanOffset += offset[i] / n;
            minOffset = Math.min(minOffset, offset[i]);
            maxOffset = Math.max(maxOffset, offset[i]);
        }

        double rightX = 0, rightY = 0;
        boolean fitted = false;
        if (maxOffset - minOffset >= MIN_FIT_SPREAD_IN) {
            double sumX = 0, sumY = 0;
            for (int i = 0; i < n; i++) {
                double d = offset[i] - meanOffset;
                sumX += d * (x[i] - meanX);
                sumY += d * (y[i] - meanY);
            }
            double length = Math.hypot(sumX, sumY);
            if (length > 0) {
                rightX = sumX / length;
                rightY = sumY / length;
                fitted = true;
            }
        }
        if (!fitted) {
            if (centerKnown) return; // don't let a rough guess spoil a good one
            double sightX = meanX - CAMERA_FORWARD_OF_CENTER_IN;
            double sightY = meanY - CAMERA_LEFT_OF_CENTER_IN;
            double length = Math.hypot(sightX, sightY);
            if (length < 1) return;
            rightX = sightY / length;
            rightY = -sightX / length;
        }

        double robotX = meanX - meanOffset * rightX;
        double robotY = meanY - meanOffset * rightY;

        // Remember it on the field so it survives the tags going out of view
        Pose pose = follower.pose();
        double cos = Math.cos(pose.heading());
        double sin = Math.sin(pose.heading());
        double fieldX = pose.x() + robotX * cos - robotY * sin;
        double fieldY = pose.y() + robotX * sin + robotY * cos;
        if (centerKnown) {
            centerFieldX += CENTER_SMOOTHING * (fieldX - centerFieldX);
            centerFieldY += CENTER_SMOOTHING * (fieldY - centerFieldY);
        } else {
            centerFieldX = fieldX;
            centerFieldY = fieldY;
            centerKnown = true;
        }
    }

    private static int slotOf(int[] ids, int id) {
        for (int i = 0; i < ids.length; i++) if (ids[i] == id) return i;
        return -1;
    }

    private static double angleDiff(double a, double b) {
        double d = a - b;
        return Math.atan2(Math.sin(d), Math.cos(d));
    }

    /*
     * Runs the windmill at full power to feed the launcher, and the intake at half of that to
     * push elements toward it, the same as the teleop's launch(). 0 stops them all.
     */
    private void setFeed(double windmillPower) {
        windmillServo.setPower(windmillPower);
        double intakePower = windmillPower * FEED_INTAKE_POWER;
        intake.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
    }

    /*
     * Updates the follower until it finishes the current path (or the opmode stops).
     */
    private void followPath(Follower follower, String step) {
        while (opModeIsActive() && follower.following()) {
            follower.update();

            telemetry.addData("Step", step);
            telemetry.addData("Distance left (in)", "%.1f", follower.distanceToEndpoint());
            addPoseTelemetry(follower);
            telemetry.update();
        }
    }

    /*
     * Runs the intake motor and corner servo to pull elements in, without the windmill, the
     * same as the right trigger in the teleop. 0 stops them.
     */
    private void setIntake(double power) {
        intake.setPower(power);
        rightIntakeServo.setPower(power);
    }

    private void addAimTelemetry(double aimErrorDeg) {
        if (limelight == null) {
            telemetry.addData("Aim", "no Limelight: %s", limelightError);
        } else if (Double.isNaN(aimErrorDeg)) {
            telemetry.addData("Aim", "no tags seen yet");
        } else {
            int[] ids = CLUSTERS[targetCluster];
            telemetry.addData("Aim", "cluster %d-%d, %.1f deg off, %d tag%s in view",
                    ids[0], ids[ids.length - 1], aimErrorDeg, tagsSeen, tagsSeen == 1 ? "" : "s");
        }
    }

    private void addPoseTelemetry(Follower follower) {
        Pose pose = follower.pose();
        telemetry.addData("Pose", "x %.1f in, y %.1f in, heading %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }
}
