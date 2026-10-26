package org.firstinspires.ftc.teamcode.Mecanismos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Shooter3 extends OpMode {

    private DcMotorEx motorShooter;

    private static final String MOTOR_NAME = "motorShooter";
    private static final double MAX_RPM = 5990.4;

    public void init(){
        motorShooter = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        motorShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    @Override
    public void loop(){
        double trigger = gamepad1.right_trigger;

        if (trigger < 0.05) {
            trigger = 0.0;
        }

        double ticksPerRevolution = motorShooter.getMotorType().getTicksPerRev();

        double targetRpm = trigger * MAX_RPM;
        double targetTicksPerSecond = targetRpm / 60.0 * ticksPerRevolution;

        if (trigger > 0.0) {
            motorShooter.setVelocity(targetTicksPerSecond);
        } else { motorShooter.setVelocity(0.0);}

        double currentTicksPerSecond = motorShooter.getVelocity();

        double currentRpm = 0.0;

        if (ticksPerRevolution > 0.0) {
            currentRpm = currentTicksPerSecond * 60.0 / ticksPerRevolution;
        }

        telemetry.addData("Trigger", "%.2f", trigger);
        telemetry.addData("RPM Alvo", "%.1f", targetRpm);
        telemetry.addData("RPM Atual", "%.1f", currentRpm);
        telemetry.addData("Ticks/s alvo", "%.1f", currentTicksPerSecond);
        telemetry.addData("Ticks/rev", "%.1f", ticksPerRevolution);
        telemetry.addData("Posição Encoder", motorShooter.getCurrentPosition());
        telemetry.addData("Potência Aplicada", "%.2f", motorShooter.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (motorShooter != null) {
            motorShooter.setVelocity(0.0);
            motorShooter.setPower(0.0);
        }
    }
}
