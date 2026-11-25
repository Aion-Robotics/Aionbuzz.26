package org.firstinspires.ftc.teamcode.Repository;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class GyroController {
    public enum TurnStatus {
        RUNNING,
        SUCCESS,
        TIMEOUT
    }

    public static class TurnOutput {
        public final TurnStatus status;
        public final double angle;
        public final double targetAngle;
        public final double error;
        public final double turnPower;
        public final double angularVelocity;
        public final int stableReadings;

        private TurnOutput(
                TurnStatus status,
                double angle,
                double targetAngle,
                double error,
                double turnPower,
                double angularVelocity,
                int stableReadings
        ) {
            this.status = status;
            this.angle = angle;
            this.targetAngle = targetAngle;
            this.error = error;
            this.turnPower = turnPower;
            this.angularVelocity = angularVelocity;
            this.stableReadings = stableReadings;
        }
    }

    private final IMU imu;

    private double continuousAngle;
    private double lastYaw;
    private double headingTarget;
    private boolean headingHoldStarted;

    private boolean turnStarted;
    private boolean turnFinished;
    private double targetAngle;
    private double filteredAngularVelocity;
    private double previousAngle;
    private long previousTimeNanos;
    private long turnTimeoutNanos;
    private int stableReadings;
    private TurnStatus turnStatus = TurnStatus.RUNNING;

    private static final double MIN_TURN_POWER = 0.10;

    public GyroController(HardwareMap hardwareMap) {
        this(
                hardwareMap,
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
    }

    public GyroController(
            HardwareMap hardwareMap,
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection
    ) {
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                logoDirection,
                usbDirection
        );

        imu.initialize(new IMU.Parameters(orientation));
        resetYaw();
    }

    public void resetYaw() {
        imu.resetYaw();
        lastYaw = readYaw();
        continuousAngle = 0.0;
        headingTarget = 0.0;
        headingHoldStarted = false;
    }

    public double getYaw() {
        return readYaw();
    }

    public double getContinuousAngle() {
        double currentYaw = readYaw();
        double delta = currentYaw - lastYaw;

        while (delta > 180.0) delta -= 360.0;
        while (delta < -180.0) delta += 360.0;

        continuousAngle += delta;
        lastYaw = currentYaw;
        return continuousAngle;
    }

    public void startHeadingHold() {
        headingTarget = getContinuousAngle();
        headingHoldStarted = true;
    }

    public void setHeadingTarget(double headingDegrees) {
        headingTarget = headingDegrees;
        headingHoldStarted = true;
    }

    public double getHeadingTarget() {
        return headingTarget;
    }

    public double getHeadingError() {
        if (!headingHoldStarted) startHeadingHold();
        return headingTarget - getContinuousAngle();
    }

    public double straightCorrection() {
        return headingCorrection(0.018, 0.12);
    }

    public double straightCorrection(double kp, double maxCorrection) {
        return headingCorrection(kp, maxCorrection);
    }

    public double straightCorrection(
            double yaw,
            double kp,
            double maxCorrection
    ) {
        return Range.clip(yaw * kp, -Math.abs(maxCorrection), Math.abs(maxCorrection));
    }

    public double headingCorrection(double kp, double maxCorrection) {
        double error = getHeadingError();
        return Range.clip(error * kp, -Math.abs(maxCorrection), Math.abs(maxCorrection));
    }

    public void startTurn(double degrees) {
        startTurn(degrees, 12.0);
    }

    public void startTurn(double degrees, double timeoutSeconds) {
        double initialAngle = getContinuousAngle();

        targetAngle = initialAngle + degrees;
        filteredAngularVelocity = 0.0;
        previousAngle = initialAngle;
        previousTimeNanos = System.nanoTime();
        turnTimeoutNanos = System.nanoTime()
                + (long) (timeoutSeconds * 1_000_000_000L);
        stableReadings = 0;
        turnStarted = true;
        turnFinished = false;
        turnStatus = TurnStatus.RUNNING;
    }

    public TurnOutput updateTurn() {
        if (!turnStarted) {
            throw new IllegalStateException("Chame startTurn() antes de updateTurn().");
        }

        if (turnFinished) {
            return output(0.0, 0.0, 0.0);
        }

        if (System.nanoTime() >= turnTimeoutNanos) {
            turnFinished = true;
            turnStatus = TurnStatus.TIMEOUT;
            return output(0.0, 0.0, 0.0);
        }

        double currentAngle = getContinuousAngle();
        double error = targetAngle - currentAngle;

        long now = System.nanoTime();
        double deltaTime = (now - previousTimeNanos) / 1_000_000_000.0;
        if (deltaTime <= 0.0) deltaTime = 0.001;

        double rawAngularVelocity = (currentAngle - previousAngle) / deltaTime;
        filteredAngularVelocity =
                0.8 * filteredAngularVelocity + 0.2 * rawAngularVelocity;

        double kp = 0.018;
        double kd = 0.0;

        double command = Range.clip(
                kp * error - kd * filteredAngularVelocity,
                -0.35,
                0.35
        );

        if (Math.abs(error) < 1.0
                && Math.abs(filteredAngularVelocity) < 4.0) {
            stableReadings++;

            if (stableReadings >= 5) {
                turnFinished = true;
                turnStatus = TurnStatus.SUCCESS;
                command = 0.0;
            }
        } else {
            stableReadings = 0;
        }

        if (!turnFinished
                && Math.abs(error) > 1.0
                && Math.abs(command) < MIN_TURN_POWER) {
            command = Math.copySign(MIN_TURN_POWER, command);
        }

        previousAngle = currentAngle;
        previousTimeNanos = now;

        return output(
                command,
                currentAngle,
                error
        );
    }

    public boolean isTurnFinished() {
        return turnFinished;
    }

    public TurnStatus getTurnStatus() {
        return turnStatus;
    }

    private TurnOutput output(double command, double angle, double error) {
        return new TurnOutput(
                turnStatus,
                angle,
                targetAngle,
                error,
                command,
                filteredAngularVelocity,
                stableReadings
        );
    }

    private double readYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}