package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled
@TeleOp
public class TestIMU extends LinearOpMode {

    @Override
    public void runOpMode() {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientacaoHub =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        imu.initialize(new IMU.Parameters(orientacaoHub));
        waitForStart();
        if (isStopRequested()){
            return;}

        imu.resetYaw();

        while (opModeIsActive()) {

            YawPitchRollAngles angulos = imu.getRobotYawPitchRollAngles();

            double yaw = angulos.getYaw(AngleUnit.DEGREES);
            double pitch = angulos.getPitch(AngleUnit.DEGREES);
            double roll = angulos.getRoll(AngleUnit.DEGREES);

            telemetry.addData("Yaw/Heading", "%.1f graus", yaw);
            telemetry.addData("Pitch", "%.1f graus", pitch);
            telemetry.addData("Roll", "%.1f graus", roll);
            telemetry.update();
        }
    }
}
