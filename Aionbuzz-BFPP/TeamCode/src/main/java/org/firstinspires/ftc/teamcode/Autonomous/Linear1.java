package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
public class Linear1 extends LinearOpMode {
    @Override
    public void runOpMode(){

        DcMotor esq = hardwareMap.get(DcMotor.class, "motorEsq");
        DcMotor dir = hardwareMap.get(DcMotor.class, "motorDir");
        dir.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        esq.setPower(0.50);
        dir.setPower(0.50);
        sleep(2250);
        stop();
        sleep(200);
        esq.setPower(-0.60);
        dir.setPower(0.60);
        sleep(250);
        stop();
        sleep(200);
        esq.setPower(0.50);
        dir.setPower(0.50);
        sleep(1000);
        stop();

        while (opModeIsActive()){
            telemetry.addData("Status", "Finalizado");
            telemetry.update();
        }
    }

}
