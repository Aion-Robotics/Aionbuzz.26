package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class Linear1 extends LinearOpMode {
    @Override
    public void runOpMode(){

        DcMotor esq = hardwareMap.get(DcMotor.class, "motorEsq");
        DcMotor dir = hardwareMap.get(DcMotor.class, "motorDir");
        dir.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        esq.setPower(0.75);
        dir.setPower(0.75);
        sleep(1500);
        esq.setPower(0);
        dir.setPower(0);

        while (opModeIsActive()){
            telemetry.addData("Status", "Finalizado");
            telemetry.update();
        }
    }

}
