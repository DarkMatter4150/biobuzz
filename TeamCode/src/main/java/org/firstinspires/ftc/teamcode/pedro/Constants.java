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
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // From PinpointTuner
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-5.47021460345411);
        c.yPodOffset.set(-3.2932083250030755);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
        // From ForesightTuner
        public static ForesightConfig foresightConfig = new ForesightConfig(
                c -> {
                    Controller primaryTranslationalForward = Controller.proportional(0.09652199526485174);
                    Controller secondaryTranslationalForward = Controller.proportional(0.03566229813514899);
                    Controller primaryTranslationalLateral = Controller.proportional(0.15611438003259268);
                    Controller secondaryTranslationalLateral = Controller.proportional(0.05768009196898177);

                    c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                    c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                    c.coast.set(Controller.proportionalFeedforward(0.0032881770589424514));
                    c.brake.set(Controller.proportionalFeedforward(0.0027949505001010834));

                    c.headingFeedback.set(Controller.proportional(3.5888700763933343));
                    c.headingBrakeCoefficients.set(Vector2D.cartesian(0.09547307615375657, 0.0059709277202785556));

                    c.linearBrakeCoefficients.set(Matrix.diag(0.004848136848758384, 0.09401187940743871));
                    c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015286256226297132, 8.019936953475674E-4));

                    c.maxAchievableForwardVelocity.set(171.11476996153868);
                    c.maxAchievableStrafeVelocity.set(147.81300895647522);
                    c.naturalForwardDeceleration.set(51.390176904580215);
                    c.naturalStrafeDeceleration.set(39.10330380383026);
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