package org.firstinspires.ftc.teamcode.Mecanismos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Shooter Test")
public class ShooterTest extends OpMode {
    private Shooter shooter;

    private static final double TEST_RPM = 3000.0;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        shooter.setTargetrpm(TEST_RPM);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper){
            shooter.update();
        } else {
            shooter.stop();
        }

        telemetry.addData("Alvo (RPM)", shooter.getTargetRpm());
        telemetry.addData("Atual (RPM)", shooter.getVelocityRpm());
        telemetry.addData("Alvo (Ticks/s)", shooter.getTargetTicksPerSecond());
        telemetry.addData("Atual (Ticks/s)", shooter.getVelocityTicksPerSecond());
        telemetry.addData("Ticks/rev", shooter.getTicksPerRevolution());
        telemetry.addData("Pronto", gamepad1.right_bumper && shooter.isAtSpeed());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooter!= null){
            shooter.stop();
        }
    }
}
