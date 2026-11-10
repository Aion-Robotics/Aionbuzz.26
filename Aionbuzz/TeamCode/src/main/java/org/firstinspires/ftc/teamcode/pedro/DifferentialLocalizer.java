package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.CustomIMU;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Timer;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DifferentialLocalizer implements Localizer {

    private final Encoder encoderEsq;
    private final Encoder encoderDir;
    private final CustomIMU imu;

    private final double ticksToInches;
    private final double trackWidth;

    private final Timer timer;

    private Pose pose = Pose.zero();
    private MotionState motionState;

    private double previousImuHeading;

    public DifferentialLocalizer(
            HardwareMap hardwareMap,
            String motorEsqName,
            String motorDirName,
            String imuName,
            double ticksToInches,
            double trackWidth
    ) {
        this.ticksToInches = ticksToInches;
        this.trackWidth = trackWidth;

        DcMotorEx motorEsq =
                hardwareMap.get(
                        DcMotorEx.class,
                        motorEsqName
                );

        DcMotorEx motorDir =
                hardwareMap.get(
                        DcMotorEx.class,
                        motorDirName
                );

        /*
         * A mesma orientação deve ser usada pelo drivetrain.
         *
         * Se o teste de encoder ficar invertido, ajustaremos
         * as direções abaixo.
         */
        encoderEsq = new Encoder(motorEsq);
        encoderDir = new Encoder(motorDir);

        encoderEsq.setDirection(Encoder.FORWARD);
        encoderDir.setDirection(Encoder.FORWARD);

        /*
         * Ajuste estas duas direções caso o teste mostre que
         * um lado está sendo contado ao contrário.
         */
        imu = new RevHubIMU(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(hardwareMap, imuName);

        timer = new Timer();

        previousImuHeading =
                Angle.normalize(imu.getHeading());

        motionState =
                MotionState.ofVelocity(
                        pose,
                        Velocity.zero()
                );

        update();
    }

    @Override
    public void setPose(Pose newPose) {
        pose = newPose;

        encoderEsq.reset();
        encoderDir.reset();

        previousImuHeading =
                Angle.normalize(imu.getHeading());

        motionState =
                MotionState.ofVelocity(
                        pose,
                        Velocity.zero()
                );
    }

    @Override
    public void update() {
        long deltaTimeNanoseconds =
                timer.nanoseconds();

        timer.reset();

        encoderEsq.update();
        encoderDir.update();

        double currentImuHeading =
                Angle.normalize(imu.getHeading());

        double deltaHeading =
                Angle.turnDirection(
                        previousImuHeading,
                        currentImuHeading
                ) * Angle.smallestDifference(
                        currentImuHeading,
                        previousImuHeading
                );

        previousImuHeading = currentImuHeading;

        double deltaTicksEsq =
                encoderEsq.getDeltaPosition();

        double deltaTicksDir =
                encoderDir.getDeltaPosition();

        double deltaDistanceEsq =
                deltaTicksEsq * ticksToInches;

        double deltaDistanceDir =
                deltaTicksDir * ticksToInches;

        /*
         * Movimento diferencial:
         *
         * avanço =
         * (distância esquerda + distância direita) / 2
         */
        double deltaForward =
                (deltaDistanceEsq + deltaDistanceDir) / 2.0;

        /*
         * Não existe movimento lateral nesta base.
         */
        double deltaLateral = 0.0;

        /*
         * A pose exponential transforma o deslocamento
         * relativo ao robô em deslocamento global.
         */
        pose = pose.exp(
                new Twist(
                        deltaForward,
                        deltaLateral,
                        deltaHeading
                )
        );

        double deltaTimeSeconds =
                deltaTimeNanoseconds / 1_000_000_000.0;

        if (deltaTimeSeconds <= 0.0) {
            deltaTimeSeconds = 0.001;
        }

        Velocity velocity =
                new Velocity(
                        deltaForward / deltaTimeSeconds,
                        deltaLateral / deltaTimeSeconds,
                        deltaHeading / deltaTimeSeconds
                );

        motionState =
                MotionState.ofVelocity(
                        pose,
                        velocity
                );
    }

    @Override
    public MotionState state() {
        return motionState;
    }

    @Override
    public void reset() {
        imu.resetYaw();

        encoderEsq.reset();
        encoderDir.reset();

        pose = Pose.zero();

        previousImuHeading =
                Angle.normalize(imu.getHeading());

        motionState =
                MotionState.ofVelocity(
                        pose,
                        Velocity.zero()
                );
    }
}
