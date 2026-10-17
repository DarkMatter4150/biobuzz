package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fL");
        c.frontRightName.set("fR");
        c.backLeftName.set("bL");
        c.backRightName.set("bR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    // From PinpointTuner
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-5.47021460345411);
        c.yPodOffset.set(-3.2932083250030755);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
        // From ForesightTuner

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.26196287943277424);
                Controller secondaryTranslationalForward = Controller.proportional(0.09678828417335492);
                Controller primaryTranslationalLateral = Controller.proportional(0.37933655976835073);
                Controller secondaryTranslationalLateral = Controller.proportional(0.14015472277485014);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.019359560651434984));
                c.brake.set(Controller.proportionalFeedforward(0.016455626553719734));

                c.headingFeedback.set(Controller.proportional(3.097006469590955));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04458654989463959, 0.00695533330868837));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07924724494946672, 0.04109608698533518));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001216997025516205, 0.0022615771038304805));

                c.maxAchievableForwardVelocity.set(54.66622218021507);
                c.maxAchievableStrafeVelocity.set(43.165461820928975);
                c.naturalForwardDeceleration.set(36.378325683950344);
                c.naturalStrafeDeceleration.set(63.78073794515867);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}