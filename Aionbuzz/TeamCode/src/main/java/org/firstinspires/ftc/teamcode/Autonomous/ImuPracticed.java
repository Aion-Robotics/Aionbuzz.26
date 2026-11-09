package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.TeleOp.DriveTrain.POTENCIA_MAXIMA;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous

public class ImuPracticed extends LinearOpMode {
    IMU imu;
    double targetAngle;
    DcMotor motorEsq, motorDir;
    static final double KP = 12;
    private double basePower = POTENCIA_MAXIMA;

    @Override
    public void runOpMode(){
        imu = hardwareMap.get(IMU.class, "imu");
        motorDir = hardwareMap.get(DcMotor.class, "motorDir");
        motorEsq = hardwareMap.get(DcMotor.class, "motorEsq");

        motorDir.setDirection(DcMotor.Direction.REVERSE);

        RevHubOrientationOnRobot orientate = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        imu.initialize(new IMU.Parameters(orientate));
        imu.resetYaw();

        waitForStart();

        targetAngle = getYawAngle();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            stabilizeMovement(basePower);
            telemetryIMU();
        }

        motorDir.setPower(0.0);
        motorEsq.setPower(0.0);
    }

    private double getYawAngle(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private void telemetryIMU(){
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        double yaw = angles.getYaw(AngleUnit.DEGREES);
        double pitch = angles.getPitch(AngleUnit.DEGREES);
        double roll = angles.getRoll(AngleUnit.DEGREES);

        telemetry.addData("Target Angle", "%.0f graus", targetAngle);
        telemetry.addData("Yaw/Heading", "%.0f graus", yaw);
        telemetry.addData("Pitch", "%.0f graus", pitch);
        telemetry.addData("Roll", "%.0f graus", roll);
        telemetry.update();
    }

    private void stabilizeMovement(double basePower) {
        this.basePower = basePower;

        double currentAngle = getYawAngle();
        double error = targetAngle - currentAngle;

        while (error > 180) error -= 360;
        while (error <= 180) error += 360;

        double correction = error * KP;

        double leftPower = basePower - correction;
        double rightPower = basePower - correction;

        leftPower = Range.clip(leftPower, +basePower, -basePower);
        rightPower = Range.clip(rightPower, -basePower, +basePower);

        motorEsq.setPower(-leftPower);
        motorDir.setPower(-rightPower);
    }
}