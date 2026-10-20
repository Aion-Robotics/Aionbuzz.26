package org.firstinspires.ftc.teamcode.Mecanismos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Shooter Test 2")
public class ShooterTest2 extends OpMode {

    private RepositoryShooter shooter;

    private static final double MAX_RPM = 5990.4;
    private static final double TRIGGER_DEADZONE = 0.05;

    private int contadorUpdate = 0;

    @Override
    public void init() {
        shooter = new RepositoryShooter(hardwareMap);
    }

    @Override
    public void loop() {
        double trigger = gamepad1.right_trigger;

        if (trigger < TRIGGER_DEADZONE) {
            trigger = 0.0;
        }

        double rpmDesejado = trigger * trigger * MAX_RPM;

        shooter.setTargetRpmFromDriver(rpmDesejado);

        if (trigger > 0.05) {
            shooter.update();
            contadorUpdate++;
        } else {
            shooter.stop();
        }

        telemetry.addData("Trigger", "%.2f", trigger);
        telemetry.addData("RPM desejado", "%.1f", rpmDesejado);
        telemetry.addData("RPM alvo", "%.1f", shooter.getTargetRpm());
        telemetry.addData("RPM atual", "%.1f", shooter.getVelocityRpm());
        telemetry.addData("Ticks/s alvo", "%.1f",
                shooter.getTargetTicksPerSecond());
        telemetry.addData("Ticks/s atual", "%.1f",
                shooter.getVelocityTicksPerSecond());
        telemetry.addData("Ticks/rev", "%.2f",
                shooter.getTicksPerRevolution());
        telemetry.addData("Max RPM SDK", "%.1f",
                shooter.getMotorMaxRpm());
        telemetry.addData("Max ticks/s SDK", "%.1f",
                shooter.getMotorMaxTicksPerSecond());

        telemetry.addData("Pronto", trigger > 0.0 && shooter.isAtSpeed());
        telemetry.update();

    }

    @Override
    public void stop() {
        if (shooter != null) {
            shooter.stop();
        }
    }

}