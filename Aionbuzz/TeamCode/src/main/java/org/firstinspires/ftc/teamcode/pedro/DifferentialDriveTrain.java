package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

public class DifferentialDriveTrain implements Drivetrain {

    private final DcMotorEx motorEsq;
    private final DcMotorEx motorDir;

    private double lastLeftPower = 0.0;
    private double lastRightPower = 0.0;

    public DifferentialDriveTrain(
            HardwareMap hardwareMap,
            String motorEsqName,
            String motorDirName
    ) {
        motorEsq = hardwareMap.get(DcMotorEx.class, motorEsqName);
        motorDir = hardwareMap.get(DcMotorEx.class, motorDirName);

        /*
         * Esta configuração é um ponto de partida.
         *
         * Com potência positiva nos dois motores, o robô deve andar
         * para frente. Se ele girar ou andar para trás, ajuste estas
         * direções.
         */
        motorEsq.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDir.setDirection(DcMotorSimple.Direction.FORWARD);

        motorEsq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorEsq.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDir.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        double forward = powers.forward();
        double turn = powers.turn();

        /*
         * O campo strafe é ignorado porque o robô não possui
         * movimento lateral.
         */
        double leftPower = forward - turn;
        double rightPower = forward + turn;

        /*
         * Normaliza para que nenhum valor ultrapasse -1.0 ou 1.0.
         */
        double maximum = Math.max(
                1.0,
                Math.max(
                        Math.abs(leftPower),
                        Math.abs(rightPower)
                )
        );

        leftPower /= maximum;
        rightPower /= maximum;

        lastLeftPower = leftPower;
        lastRightPower = rightPower;

        motorEsq.setPower(leftPower);
        motorDir.setPower(rightPower);
    }

    @Override
    public double maxScaling(
            DrivePowers current,
            DrivePowers delta
    ) {
        double currentLeft =
                current.forward() - current.turn();

        double currentRight =
                current.forward() + current.turn();

        double deltaLeft =
                delta.forward() - delta.turn();

        double deltaRight =
                delta.forward() + delta.turn();

        double scale = 1.0;

        scale = calculateScale(
                currentLeft,
                deltaLeft,
                scale
        );

        scale = calculateScale(
                currentRight,
                deltaRight,
                scale
        );

        return Math.max(0.0, Math.min(1.0, scale));
    }

    private double calculateScale(
            double current,
            double delta,
            double currentScale
    ) {
        if (Math.abs(delta) < 1e-9) {
            return currentScale;
        }

        double scaleToPositiveLimit =
                (1.0 - current) / delta;

        double scaleToNegativeLimit =
                (-1.0 - current) / delta;

        double result = currentScale;

        if (scaleToPositiveLimit >= 0.0
                && scaleToPositiveLimit < result) {
            result = scaleToPositiveLimit;
        }

        if (scaleToNegativeLimit >= 0.0
                && scaleToNegativeLimit < result) {
            result = scaleToNegativeLimit;
        }

        return result;
    }

    @Override
    public void stop() {
        stop(true);
    }

    @Override
    public void stop(boolean brake) {
        motorEsq.setZeroPowerBehavior(
                brake
                        ? DcMotor.ZeroPowerBehavior.BRAKE
                        : DcMotor.ZeroPowerBehavior.FLOAT
        );

        motorDir.setZeroPowerBehavior(
                brake
                        ? DcMotor.ZeroPowerBehavior.BRAKE
                        : DcMotor.ZeroPowerBehavior.FLOAT
        );

        motorEsq.setPower(0.0);
        motorDir.setPower(0.0);

        lastLeftPower = 0.0;
        lastRightPower = 0.0;
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> data = new HashMap<>();

        data.put("leftPower", lastLeftPower);
        data.put("rightPower", lastRightPower);

        return data;
    }

    @Override
    public double interpolateVelocity(
            double xRadius,
            double yRadius,
            double theta
    ) {
        /*
         * A base diferencial só se movimenta no eixo forward.
         * O valor de yRadius não representa um movimento real aqui.
         */
        return xRadius;
    }
}
