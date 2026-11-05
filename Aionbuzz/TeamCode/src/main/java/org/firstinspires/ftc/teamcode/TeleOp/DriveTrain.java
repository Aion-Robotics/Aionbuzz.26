package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "ArcadeDrive")
public class DriveTrain extends OpMode {
    DcMotor[] motor = new DcMotor[3];
    public static final double POTENCIA_MAXIMA = 0.5;

    @Override
    public void init(){
        initHardware();
        telemetry.addData("Hardware Status", "Registered");
    }

    @Override
    public void loop(){
        // Odometria
        double totalDistance = getCurrentDistance();
        telemetry.addData("Distance", "%.1f cm", totalDistance);

        driveRobot();
        runIntakeControl();
    }

    private void initHardware(){
        // Registra os motores do Drive Train
        motor[0] = hardwareMap.get(DcMotor.class, "motorDir");
        motor[1] = hardwareMap.get(DcMotor.class, "motorEsq");
        motor[2] = hardwareMap.get(DcMotor.class, "motorIntake");

        // Reverte o sentido de um dos motores da base
        motor[1].setDirection(DcMotor.Direction.REVERSE);
    }
    private double getCurrentDistance(){
        // Calcula a distância percorrida por ambos os motores juntos
        Double[] distance = new Double[2];
        for (int i = 0; i < 2; i++) {
            int ticks = motor[i].getCurrentPosition();
            double cm = (ticks / 537.6) * (9.66 * Math.PI);
            distance[i] = cm;
        }
        return (distance[0] + distance[1]) / 2.0;
    }
    private void driveRobot(){
        DcMotor.ZeroPowerBehavior stateMotor;
        double currentMax = POTENCIA_MAXIMA;

        // Recebe os valores dos analógicos do controle
        double stickY = -gamepad1.left_stick_y;
        double stickX = gamepad1.left_stick_x;

        // Se o botão RB é pressionado, os motores funcionam na capacidade máxima
        if (gamepad1.right_bumper)
            currentMax = 1.0;

        // Guarda as potências individuais de cada motor
        double leftMove = Range.clip(stickY - stickX, -currentMax, currentMax);
        double rightMove = Range.clip(stickY + stickX, -currentMax, currentMax);

        motor[0].setPower(rightMove);
        motor[1].setPower(leftMove);

        // Se ambos os motores estiverem parados, as rodas são travadas
        stateMotor = ((leftMove == 0.0) && (rightMove == 0.0)) ?
                DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;

        for (int i = 0; i < 2; i++)
            motor[i].setZeroPowerBehavior(stateMotor);
    }
    private void runIntakeControl(){
        boolean intakeLigado;
        if (gamepad1.left_bumper) { // Se o botão LB estiver pressionado, o intake é ligado
            motor[2].setPower(0.75);
            intakeLigado = true;
        }
        else { // Caso contrário, ele permanece desligado
            motor[2].setPower(0.0);
            intakeLigado = false;
        }
        telemetry.addData("Intake Status", intakeLigado ? "On" : "Off");
    }
}