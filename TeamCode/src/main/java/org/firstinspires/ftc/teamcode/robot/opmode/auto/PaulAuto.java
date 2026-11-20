package org.firstinspires.ftc.teamcode.robot.opmode.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.config.FrontWedge;

@Autonomous(name = "PaulAuto", group = "zTest")
@Disabled
public class PaulAuto extends LinearOpMode {

    private Follower follower;
    private FrontWedge frontWedge;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startRedGarden = poseFactory.of(36.2323, 8.4268, 90);
    private final Pose point1 = poseFactory.of(10.0556, 105.1716, 0);

    /*
     * Autonomous routine.
     *
     * FRONT WEDGE: it doesn't move at INIT. At START it holds the state the last opmode left it
     * in (stowed or deployed) and stays there unless you add it here. To use it, put
     * these steps in the sequence where you need them (instant and waitMs come from
     * com.pedropathing.ivy.commands.Commands):
     *
     *     instant(frontWedge::deploy),   // swing the wedge out
     *     waitMs(500),                   // optional: give the servos time to get there first
     *     follow(follower, path1()),
     *     instant(frontWedge::stow),     // swing it back in
     *
     * Leave out the waitMs if the robot can start driving while the wedge is still moving.
     */
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        frontWedge = new FrontWedge(hardwareMap); // doesn't move at INIT
        follower.setPose(startRedGarden);
        follower.update();

        waitForStart();
        frontWedge.holdRememberedState(); // stays stowed/deployed as the last opmode left it
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return line(startRedGarden, point1).linear(startRedGarden, point1);
    }
}
