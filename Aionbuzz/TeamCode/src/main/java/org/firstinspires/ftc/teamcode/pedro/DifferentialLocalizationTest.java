package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Pedro Differential Localization Test")
public class DifferentialLocalizationTest extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);

        telemetry.addLine("Robô pronto");
        telemetry.addLine("Não pressione START até o robô estar seguro");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData(
                    "Heading rad",
                    follower.pose().heading()
            );
            telemetry.addData(
                    "Heading graus",
                    Math.toDegrees(
                            follower.pose().heading()
                    )
            );

            telemetry.update();
        }
    }
}
