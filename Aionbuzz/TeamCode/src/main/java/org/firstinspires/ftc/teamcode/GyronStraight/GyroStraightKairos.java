package org.firstinspires.ftc.teamcode.GyronStraight;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class GyroStraightKairos extends LinearOpMode {

    private DcMotor motorDir;
    private DcMotor motorEsq;
    private IMU imu;

    // Dados das rodas e encoders
    private double ticksPorVolta = 537.6;
    private double diametroRoda = 9.6;
    private double circunferencia = Math.PI * diametroRoda;
    private double anguloContinuo = 0;
    private double ultimoYaw = 0;

    @Override
    public void runOpMode() {

        // 1. Configuração dos motores
        motorDir = hardwareMap.get(DcMotor.class, "motorDir");
        motorEsq = hardwareMap.get(DcMotor.class, "motorEsq");

        motorEsq.setDirection(DcMotor.Direction.REVERSE);

        motorDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorEsq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2. Configuração do IMU
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientacao =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                );

        imu.initialize(new IMU.Parameters(orientacao));

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // 3. Executa: andar 100 cm com correção do giroscópio
        boolean terminou = gyroStraight(150, 0.15, -0.012);

        pararMotores();

        telemetry.addData("Resultado", terminou
                ? "Distancia atingida"
                : "Movimento interrompido");
        telemetry.update();
    }

    // MÉTODO PRINCIPAL: GyroStraight
    private boolean gyroStraight(
            double distancia,
            double velocidade,
            double kp) {

        // BLOCO: zerar ângulo do giroscópio
        imu.resetYaw();

        // BLOCO: zerar posição relativa dos motores
        motorDir.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEsq.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDir.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorEsq.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // BLOCO: definir distância desejada
        double distanciaAlvo = distancia;

        resetRuntime();

        // BLOCO: repetir até atingir a distância
        while (opModeIsActive() && getRuntime() < 15) {

            // Ler encoders
            double ticksDir = Math.abs(motorDir.getCurrentPosition());
            double ticksEsq = Math.abs(motorEsq.getCurrentPosition());

            double ticksMedios = (ticksDir + ticksEsq) / 2.0;

            // Converter ticks em centímetros
            double distanciaAtual =
                    (ticksMedios / ticksPorVolta) * circunferencia;

            // Verificar se chegou ao destino
            if (distanciaAtual >= distanciaAlvo) {
                pararMotores();
                return true;
            }

            // Ler ângulo do IMU
            double angulo = imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.DEGREES);

            // BLOCO: Correction = Gyro angle * KP
            double correcao = angulo * kp;

            correcao = Range.clip(correcao, -0.15, 0.15);

            // BLOCO: corrigir potência dos motores
            double potenciaDir = velocidade - correcao;
            double potenciaEsq = velocidade + correcao;

            motorDir.setPower(Range.clip(potenciaDir, -1, 1));
            motorEsq.setPower(Range.clip(potenciaEsq, -1, 1));

            // Telemetry
            telemetry.addData("Distancia atual", distanciaAtual);
            telemetry.addData("Distancia alvo", distanciaAlvo);
            telemetry.addData("Angulo IMU", angulo);
            telemetry.addData("Correcao", correcao);
            telemetry.addData("Motor direito", potenciaDir);
            telemetry.addData("Motor esquerdo", potenciaEsq);
            telemetry.update();
        }

        // Se atingir o tempo limite ou receber pedido de parada
        pararMotores();
        return false;
    }

    private void pararMotores() {
        motorDir.setPower(0);
        motorEsq.setPower(0);
    }
}