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
        shooter.setTargetRpm(TEST_RPM);
    }

    @Override
    public void loop() {
        boolean gatilhoPressionado = gamepad1.right_trigger > 0.05;


        if (gatilhoPressionado){
            shooter.update();
        } else {
            shooter.stop();
        }

        telemetry.addData("Gatilho", "%.2f", gamepad1.right_trigger);
        telemetry.addData("Alvo (RPM)", "%.1f", shooter.getTargetRpm());
        telemetry.addData("Atual (RPM)", "%.1f", shooter.getVelocityRpm());
        telemetry.addData("Alvo (Ticks/s)", "%.1f",
                shooter.getTargetTicksPerSecond());
        telemetry.addData("Atual (Ticks/s)", "%.1f",
                shooter.getVelocityTicksPerSecond());
        telemetry.addData("Ticks/rev", "%.1f",
                shooter.getTicksPerRevolution());
        telemetry.addData("Pronto",
                gatilhoPressionado && shooter.isAtSpeed());

        telemetry.update();
    }

    @Override
    public void stop() {
        motor.setVelocity(0.0);
        motor.setPower(0.0);
        }
    }

