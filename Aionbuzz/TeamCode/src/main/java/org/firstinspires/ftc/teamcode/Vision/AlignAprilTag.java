package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Repository.RepositoryAprilTag;


@TeleOp
public class AlignAprilTag extends OpMode {

    private RepositoryAprilTag aprilTag;
    private final ElapsedTime cronometroLoop = new ElapsedTime();

    private boolean modoAutomatico;
    private boolean ultimoA;
    private boolean ultimoUp;
    private boolean ultimoRight;
    private boolean ultimoDown;
    private boolean ultimoLeft;

    @Override
    public void init() {
        aprilTag = new RepositoryAprilTag(hardwareMap);
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("AlignAprilTag iniciado");
        telemetry.addLine("A = alterna Manual/Alinhamento");
        telemetry.addLine("D-pad = seleciona a Hive");
        telemetry.update();
    }

    @Override
    public void start() {
        aprilTag.start();
        modoAutomatico = false;
        ultimoA = false;
        cronometroLoop.reset();
    }

    @Override
    public void loop() {
        selecionarHive();
        processarBotaoModo();

        if (modoAutomatico) {
            double deltaTime = cronometroLoop.seconds();
            cronometroLoop.reset();
            aprilTag.updateAlignment(telemetry, deltaTime);
        } else {
            cronometroLoop.reset();
            dirigirManualmente();
            telemetry.addData("Modo", "MANUAL");
        }

        adicionarTelemetriaAprilTag();
        atualizarShooter();
        atualizarIntake();
        telemetry.update();
    }

    private void processarBotaoModo() {
        if (gamepad1.a && !ultimoA) {
            modoAutomatico = !modoAutomatico;

            if (modoAutomatico) {
                aprilTag.startAlignment();
            } else {
                aprilTag.cancelAlignment();
            }
        }
        ultimoA = gamepad1.a;
    }

    private void selecionarHive() {
        if (gamepad1.dpad_up && !ultimoUp) {
            aprilTag.setHive(RepositoryAprilTag.Hive.RED_SCORING);
        }
        if (gamepad1.dpad_right && !ultimoRight) {
            aprilTag.setHive(RepositoryAprilTag.Hive.RED_AUDIENCE);
        }
        if (gamepad1.dpad_down && !ultimoDown) {
            aprilTag.setHive(RepositoryAprilTag.Hive.BLUE_AUDIENCE);
        }
        if (gamepad1.dpad_left && !ultimoLeft) {
            aprilTag.setHive(RepositoryAprilTag.Hive.BLUE_SCORING);
        }

        ultimoUp = gamepad1.dpad_up;
        ultimoRight = gamepad1.dpad_right;
        ultimoDown = gamepad1.dpad_down;
        ultimoLeft = gamepad1.dpad_left;
    }

    private void dirigirManualmente() {
        double frente = -gamepad1.left_stick_y;
        double giro = -gamepad1.left_stick_x;
        double velocidade = gamepad1.right_bumper ? 1.0 : 0.5;
        aprilTag.setDrivePower(frente * velocidade, giro * velocidade);
    }

    private void adicionarTelemetriaAprilTag() {
        RepositoryAprilTag.DebugSnapshot debug =
                aprilTag.getDebugSnapshot();

        telemetry.addData("Modo", modoAutomatico
                ? "AUTOMÁTICO" : "MANUAL");
        telemetry.addData("Hive", aprilTag.getHiveSelecionada());
        telemetry.addData("Alvo encontrado", debug.alvoEncontrado);
        telemetry.addData("Status", debug.status);
        telemetry.addData("Bearing", "%.2f graus", debug.bearing);
        telemetry.addData("Erro angular", "%.2f graus", debug.erroBearing);
        telemetry.addData("Range", "%.2f in", debug.range);
        telemetry.addData("Erro distância", "%.2f in", debug.erroRange);
        telemetry.addData("Potência avanço", "%.3f", debug.potenciaAvanco);
        telemetry.addData("Potência giro", "%.3f", debug.potenciaGiro);
        telemetry.addData("Ciclos estáveis", debug.ciclosEstaveis);
        telemetry.addData("Ciclos sem alvo", debug.ciclosSemAlvo);
        telemetry.addData("Tempo automático", "%.2f s",
                debug.tempoAlinhamento);
        telemetry.addData("Motivo saída", aprilTag.getMotivoSaida());
    }

    private void atualizarShooter() {
    }

    private void atualizarIntake() {
    }

    @Override
    public void stop() {
        if (aprilTag != null) {
            aprilTag.stop();
        }
    }
}