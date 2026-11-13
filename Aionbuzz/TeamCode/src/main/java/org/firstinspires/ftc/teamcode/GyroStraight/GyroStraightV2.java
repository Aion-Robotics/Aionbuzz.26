package org.firstinspires.ftc.teamcode.GyroStraight;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Gyro Straight 150cm", group = "Autonomous")
public class GyroStraightV2 extends LinearOpMode {

    private DcMotor motorDir;
    private DcMotor motorEsq;
    private IMU imu;

    // Dados das rodas e dos encoders
    private static final double TICKS_POR_VOLTA = 537.6;
    private static final double DIAMETRO_RODA_CM = 9.6;
    private static final double CIRCUNFERENCIA_RODA_CM =
            Math.PI * DIAMETRO_RODA_CM;

    @Override
    public void runOpMode() {
        // Mapeamento dos motores
        motorDir = hardwareMap.get(DcMotor.class, "motorDir");
        motorEsq = hardwareMap.get(DcMotor.class, "motorEsq");

        // Inverta este motor caso os dois lados não avancem na mesma direção.
        motorEsq.setDirection(DcMotor.Direction.REVERSE);

        motorDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorEsq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset e configuração dos encoders
        motorDir.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEsq.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDir.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorEsq.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Configuração do IMU conforme a orientação física da REV Hub.
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientacao = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu.initialize(new IMU.Parameters(orientacao));
        imu.resetYaw();

        telemetry.addLine("Gyro Straight 150 cm");
        telemetry.addLine("Aguardando início...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            pararMotores();
            return;
        }

        gyroStraight(150.0);
        pararMotores();

        telemetry.addLine("Deslocamento finalizado");
        telemetry.update();
        sleep(2000);
    }

    /**
     * Move o robô em linha reta usando os encoders para a distância
     * e o IMU para corrigir o desvio angular.
     */
    private void gyroStraight(double distanciaAlvoCm) {
        double ticksAlvo = (distanciaAlvoCm / CIRCUNFERENCIA_RODA_CM)
                * TICKS_POR_VOLTA;

        // Ganho da correção angular. Ajuste durante os testes do robô.
        double kp = -0.012;

        while (opModeIsActive()) {
            double ticksDir = Math.abs(motorDir.getCurrentPosition());
            double ticksEsq = Math.abs(motorEsq.getCurrentPosition());
            double ticksPercorridos = (ticksDir + ticksEsq) / 2.0;

            double distanciaPercorridaCm =
                    (ticksPercorridos / TICKS_POR_VOLTA)
                            * CIRCUNFERENCIA_RODA_CM;
            double distanciaRestanteCm =
                    distanciaAlvoCm - distanciaPercorridaCm;

            if (ticksPercorridos >= ticksAlvo) {
                break;
            }

            double angulo = imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.DEGREES);

            double correcao = Range.clip(angulo * kp, -0.15, 0.15);

            // Reduz a velocidade nos últimos 30 cm.
            double velocidade = Range.clip(
                    0.15 + (distanciaRestanteCm / 30.0) * 0.15,
                    0.15,
                    0.30
            );

            double potenciaDir = Range.clip(
                    velocidade - correcao, -1.0, 1.0);
            double potenciaEsq = Range.clip(
                    velocidade + correcao, -1.0, 1.0);

            motorDir.setPower(potenciaDir);
            motorEsq.setPower(potenciaEsq);

            telemetry.addData("Distância percorrida (cm)", "%.1f", distanciaPercorridaCm);
            telemetry.addData("Distância restante (cm)", "%.1f", distanciaRestanteCm);
            telemetry.addData("Ticks direita", ticksDir);
            telemetry.addData("Ticks esquerda", ticksEsq);
            telemetry.addData("Ângulo IMU (graus)", "%.2f", angulo);
            telemetry.addData("Correção", "%.3f", correcao);
            telemetry.addData("Potência direita", "%.3f", potenciaDir);
            telemetry.addData("Potência esquerda", "%.3f", potenciaEsq);
            telemetry.update();

            idle();
        }

        pararMotores();
    }

    private void pararMotores() {
        motorDir.setPower(0.0);
        motorEsq.setPower(0.0);
    }
}