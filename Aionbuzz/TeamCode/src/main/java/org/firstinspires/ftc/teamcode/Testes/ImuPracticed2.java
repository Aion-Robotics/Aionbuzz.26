package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class ImuPracticed2 extends LinearOpMode {

    IMU imu;
    DcMotor motorEsq, motorDir;

    final static double KP = 0.03;
    double targetAngle = 0.0;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        motorDir = hardwareMap.get(DcMotor.class, "motorDir");
        motorEsq = hardwareMap.get(DcMotor.class, "motorEsq");

        motorDir.setDirection(DcMotorSimple.Direction.REVERSE);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        )));

        imu.resetYaw();

        waitForStart();

        targetAngle = getYawAngle();

        while (opModeIsActive()) {
            stabilizeMovement(0.4);
            telemetry.addData("Alvo (Target)", targetAngle);
            telemetry.addData("Atual (Yaw)", getYawAngle());
        }

        motorEsq.setPower(0);
        motorDir.setPower(0);
    }

    private double getYawAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private void stabilizeMovement(double basePower) {
        double currentAngle = getYawAngle();
        double error = targetAngle - currentAngle;
        double correction = error * KP;

        double leftPower  = basePower - correction;
        double rightPower = basePower + correction;

        leftPower  = Range.clip(leftPower, -0.4, 0.4);
        rightPower = Range.clip(rightPower, -0.4, 0.4);

        motorEsq.setPower(leftPower);
        motorDir.setPower(rightPower);
    }
}