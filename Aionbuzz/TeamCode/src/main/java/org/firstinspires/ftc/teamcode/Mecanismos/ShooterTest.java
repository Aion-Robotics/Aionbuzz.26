package org.firstinspires.ftc.teamcode.Mecanismos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooter Test")
public class ShooterTest extends OpMode {
    private RepositoryShooter shooter;
    DcMotorEx motor;

    private static final double TEST_RPM = 5990.4;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motorShooter");
        shooter = new RepositoryShooter(hardwareMap);
        shooter.setTargetrpm(TEST_RPM);
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger_pressed){
            shooter.update();
        } else {
            shooter.stop();
        }

        telemetry.addData("Alvo (RPM)", shooter.getTargetRpm());
        telemetry.addData("Atual (RPM)", shooter.getVelocityRpm());
        telemetry.addData("Alvo (Ticks/s)", shooter.getTargetTicksPerSecond());
        telemetry.addData("Atual (Ticks/s)", shooter.getVelocityTicksPerSecond());
        telemetry.addData("Ticks/rev", shooter.getTicksPerRevolution());
        telemetry.addData("Pronto", gamepad1.right_trigger_pressed && shooter.isAtSpeed());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooter!= null){
            shooter.stop();
        }
    }
}
