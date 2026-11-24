package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Repository.AprilTagAlignmentMath;
import org.firstinspires.ftc.teamcode.Repository.GyroController;
import org.firstinspires.ftc.teamcode.Repository.IntakeTransferController;
import org.firstinspires.ftc.teamcode.Repository.RepositoryAprilTag;
import org.firstinspires.ftc.teamcode.Repository.ShooterController;

public class AutonomousV1 extends OpMode {

    private enum Estado {
        PREPARAR,
        PONTO_INICIAL,
        ALINHAR_INICIAL,
        SHOOTER_INICIAL,
        DISPARAR_PRELOAD,
        AREA_COLETA,
        COLETAR_POLENS,
        SAIR_COLETA,
        GIRAR_APOS_COLETA,
        PONTO_FINAL,
        ALINHAR_FINAL,
        SHOOTER_FINAL,
        TRANSFERIR_POLENS,
        DISPARAR_POLENS,
        ESTACIONAMENTO,
        FINALIZADO,
        ABORTADO
    }

    private enum PresetShooter {
        PRIMEIRO,
        FINAL
    }

    private DcMotor motorDir;
    private DcMotor motorEsq;
    private GyroController gyro;

    private RepositoryAprilTag repositoryAprilTag;
    private IntakeTransferController intakeTransfer;
    private ShooterController shooter;

    private Estado estado = Estado.PREPARAR;
    private final ElapsedTime tempoEstado = new ElapsedTime();
    private final ElapsedTime tempoLoop = new ElapsedTime();

    private boolean movimentoConfigurado;
    private boolean alinhamentoIniciado;
    private boolean giroIniciado;
    private boolean presetShooterSelecionado;
    private int alvoDir;
    private int alvoEsq;
    private int polensAlimentados;
    private int polensParaDisparar;
    private String motivoAbortado = "";

    private static final double TICKS_POR_VOLTA = 537.6;
    private static final double DIAMETRO_RODA_CM = 9.6;
    private static final double LARGURA_BASE_CM = 32.6;
    private static final double TICKS_POR_CM = TICKS_POR_VOLTA / (Math.PI * DIAMETRO_RODA_CM);

    private static final double TILE_CM = 60.95;
    private static final double CAMPO_CM = 6.0 * TILE_CM;
    private static final double PROFUNDIDADE_LADO_VERMELHO_CM = 3.0 * TILE_CM;

    private static final RepositoryAprilTag.Hive HIVE_ALVO =
            RepositoryAprilTag.Hive.RED_AUDIENCE;

    private static final double DIST_INICIAL_CM = 1.0 * TILE_CM;
    private static final double HEADING_INICIAL_GRAUS = 0.0;
    private static final double DIST_ATE_COLETA_CM = 1.0 * TILE_CM;
    private static final double DIST_SAIDA_COLETA_CM = 0.5 * TILE_CM;
    private static final double HEADING_SAIDA_COLETA_GRAUS = 180.0;
    private static final double DIST_ATE_PONTO_FINAL_CM = 1.0 * TILE_CM;
    private static final double HEADING_PONTO_FINAL_GRAUS = 180.0;
    private static final double DIST_ESTACIONAMENTO_CM = 0.5 * TILE_CM;

    private static final double POTENCIA_DESLOCAMENTO = 0.42;
    private static final double POTENCIA_ESTACIONAMENTO = 0.30;
    private static final double TIMEOUT_DESLOCAMENTO = 5.0;
    private static final double TIMEOUT_GIRO = 3.0;
    private static final double TIMEOUT_SPINUP = 2.5;
    private static final double TIMEOUT_TRANSFERENCIA = 2.5;

    private static final int POLENS_PRELOAD = 4;
    private static final int POLENS_A_COLETAR = 4;
    private static final int POLENS_POR_CICLO = 4;
    private static final double TEMPO_COLETA_SEGUNDOS = 2.5;
    private static final double TEMPO_TRANSFERENCIA_POR_POLEN = 0.24;
    private static final double TEMPO_PAUSA_ENTRE_POLENS = 0.28;
    private static final double TEMPO_ASSENTAR_TRANSFERENCIA = 0.20;

    @Override
    public void init() {
        inicializarHardware();
        mudarEstado(Estado.PREPARAR);
        telemetry.addData("Status", "Inicializado");
        telemetry.addData("Ticks/cm", "%.3f", TICKS_POR_CM);
        telemetry.addData("Campo nominal", "%.2f cm", CAMPO_CM);
        telemetry.addData("Lado vermelho", "%.2f cm", PROFUNDIDADE_LADO_VERMELHO_CM);
        telemetry.addData("Hive", HIVE_ALVO);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        atualizarTelemetry();
    }

    @Override
    public void start() {
        gyro.resetYaw();
        resetarEncoders();
        repositoryAprilTag.start();
        mudarEstado(Estado.PONTO_INICIAL);
    }

    @Override
    public void loop() {
        double deltaTime = tempoLoop.seconds();
        tempoLoop.reset();
        if (deltaTime <= 0.0 || deltaTime > 0.5) deltaTime = 0.02;

        switch (estado) {
            case PREPARAR:
                executarPreparar();
                break;
            case PONTO_INICIAL:
                executarDeslocamento(DIST_INICIAL_CM, HEADING_INICIAL_GRAUS,
                        POTENCIA_DESLOCAMENTO, Estado.ALINHAR_INICIAL);
                break;
            case ALINHAR_INICIAL:
                executarAlinhamento(Estado.SHOOTER_INICIAL, deltaTime);
                break;
            case SHOOTER_INICIAL:
                executarSpinup(PresetShooter.PRIMEIRO, Estado.DISPARAR_PRELOAD);
                break;
            case DISPARAR_PRELOAD:
                executarDisparos(POLENS_PRELOAD, Estado.AREA_COLETA);
                break;
            case AREA_COLETA:
                executarDeslocamento(DIST_ATE_COLETA_CM, HEADING_INICIAL_GRAUS,
                        POTENCIA_DESLOCAMENTO, Estado.COLETAR_POLENS);
                break;
            case COLETAR_POLENS:
                executarColeta();
                break;
            case SAIR_COLETA:
                iniciarShooterFinal();
                executarDeslocamento(DIST_SAIDA_COLETA_CM, HEADING_SAIDA_COLETA_GRAUS,
                        POTENCIA_DESLOCAMENTO, Estado.GIRAR_APOS_COLETA);
                break;
            case GIRAR_APOS_COLETA:
                atualizarShooter();
                executarGiro(180.0, Estado.PONTO_FINAL);
                break;
            case PONTO_FINAL:
                atualizarShooter();
                executarDeslocamento(DIST_ATE_PONTO_FINAL_CM, HEADING_PONTO_FINAL_GRAUS,
                        POTENCIA_DESLOCAMENTO, Estado.ALINHAR_FINAL);
                break;
            case ALINHAR_FINAL:
                atualizarShooter();
                executarAlinhamento(Estado.SHOOTER_FINAL, deltaTime);
                break;
            case SHOOTER_FINAL:
                executarSpinup(PresetShooter.FINAL, Estado.TRANSFERIR_POLENS);
                break;
            case TRANSFERIR_POLENS:
                executarTransferencia();
                break;
            case DISPARAR_POLENS:
                executarDisparos(polensParaDisparar, Estado.ESTACIONAMENTO);
                break;
            case ESTACIONAMENTO:
                executarDeslocamento(DIST_ESTACIONAMENTO_CM, HEADING_PONTO_FINAL_GRAUS,
                        POTENCIA_ESTACIONAMENTO, Estado.FINALIZADO);
                break;
            case FINALIZADO:
                pararTudo();
                break;
            case ABORTADO:
                pararTudo();
                break;
        }

        atualizarTelemetry();
    }

    @Override
    public void stop() {
        pararTudo();
        if (repositoryAprilTag != null) repositoryAprilTag.stop();
    }

    private void inicializarHardware() {
        motorDir = hardwareMap.get(DcMotor.class, "motorDir");
        motorEsq = hardwareMap.get(DcMotor.class, "motorEsq");
        gyro = new GyroController(hardwareMap);
        motorEsq.setDirection(DcMotor.Direction.REVERSE);
        motorDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorEsq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDir.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorEsq.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        repositoryAprilTag = new RepositoryAprilTag(hardwareMap);
        repositoryAprilTag.setHive(HIVE_ALVO);
        intakeTransfer = new IntakeTransferController(hardwareMap);
        shooter = new ShooterController(hardwareMap);
        intakeTransfer.parar();
        shooter.parar();
    }

    private void executarPreparar() {
        pararTudo();
        polensAlimentados = 0;
        polensParaDisparar = 0;
        mudarEstado(Estado.PONTO_INICIAL);
    }

    private void executarDeslocamento(double distanciaCm, double headingAlvo,
                                      double potencia, Estado proximo) {
        if (!movimentoConfigurado) {
            int deltaTicks = centimetrosParaTicks(distanciaCm);
            alvoDir = motorDir.getCurrentPosition() + deltaTicks;
            alvoEsq = motorEsq.getCurrentPosition() + deltaTicks;
            motorDir.setTargetPosition(alvoDir);
            motorEsq.setTargetPosition(alvoEsq);
            motorDir.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorEsq.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gyro.startHeadingHold();
            movimentoConfigurado = true;
        }

        double correcao = gyro.straightCorrection();
        motorEsq.setPower(Range.clip(Math.abs(potencia) + correcao, 0.0, 1.0));
        motorDir.setPower(Range.clip(Math.abs(potencia) - correcao, 0.0, 1.0));

        boolean terminou = !motorDir.isBusy() && !motorEsq.isBusy();
        boolean expirou = tempoEstado.seconds() >= TIMEOUT_DESLOCAMENTO;
        if (terminou || expirou) {
            pararDrive();
            voltarParaRunUsingEncoder();
            if (expirou && !terminou) abortar("Timeout durante deslocamento");
            else mudarEstado(proximo);
        }
    }

    private void executarGiro(double graus, Estado proximo) {
        if (!giroIniciado) {
            gyro.startTurn(graus, TIMEOUT_GIRO);
            giroIniciado = true;
        }
        GyroController.TurnOutput output = gyro.updateTurn();
        motorDir.setPower(-output.turnPower);
        motorEsq.setPower(output.turnPower);
        if (output.status == GyroController.TurnStatus.SUCCESS) {
            pararDrive();
            mudarEstado(proximo);
        } else if (output.status == GyroController.TurnStatus.TIMEOUT) {
            pararDrive();
            abortar("Timeout no giro de " + graus + " graus");
        }
    }

    private void executarAlinhamento(Estado proximo, double deltaTime) {
        if (!alinhamentoIniciado) {
            repositoryAprilTag.setHive(HIVE_ALVO);
            gyro.startHeadingHold();
            repositoryAprilTag.startAlignment();
            alinhamentoIniciado = true;
        }

        double correcaoHeading = gyro.headingCorrection(0.018, 0.12);
        repositoryAprilTag.updateAlignment(telemetry, deltaTime, correcaoHeading);
        RepositoryAprilTag.DebugSnapshot snapshot = repositoryAprilTag.getDebugSnapshot();

        if (snapshot != null && snapshot.status == AprilTagAlignmentMath.Status.PRONTO) {
            repositoryAprilTag.cancelAlignment();
            mudarEstado(proximo);
            return;
        }

        if (!repositoryAprilTag.isAlignmentActive()) {
            String motivo = repositoryAprilTag.getMotivoSaida();
            abortar(motivo == null || motivo.isEmpty()
                    ? "Alinhamento AprilTag encerrado sem sucesso" : motivo);
        }
    }

    private void executarColeta() {
        intakeTransfer.coletar(1.0, 1.0);
        shooter.parar();
        presetShooterSelecionado = false;
        if (tempoEstado.seconds() >= TEMPO_COLETA_SEGUNDOS) {
            intakeTransfer.parar();
            polensParaDisparar = Math.min(POLENS_A_COLETAR, POLENS_POR_CICLO);
            polensAlimentados = 0;
            mudarEstado(Estado.SAIR_COLETA);
        }
    }

    private void iniciarShooterFinal() {
        if (!presetShooterSelecionado) {
            shooter.selecionarPresetX();
            presetShooterSelecionado = true;
        }
        shooter.update();
    }

    private void atualizarShooter() {
        iniciarShooterFinal();
    }

    private void executarSpinup(PresetShooter preset, Estado proximo) {
        if (!presetShooterSelecionado) {
            if (preset == PresetShooter.PRIMEIRO) shooter.selecionarPresetB();
            else shooter.selecionarPresetX();
            presetShooterSelecionado = true;
        }

        shooter.update();
        if (shooter.isVelocidadeAtingida()) {
            polensAlimentados = 0;
            mudarEstado(proximo);
        } else if (tempoEstado.seconds() >= TIMEOUT_SPINUP) {
            abortar("Shooter nao atingiu RPM");
        }
    }

    private void executarTransferencia() {
        shooter.update();
        if (!shooter.isVelocidadeAtingida() || !shooter.isTravaAberta()) {
            intakeTransfer.parar();
            if (tempoEstado.seconds() >= TIMEOUT_TRANSFERENCIA) {
                abortar("Trava do shooter nao abriu");
            }
            return;
        }

        intakeTransfer.ligarTransfer(1.0);
        if (tempoEstado.seconds() >= TEMPO_ASSENTAR_TRANSFERENCIA) {
            intakeTransfer.parar();
            polensAlimentados = 0;
            mudarEstado(Estado.DISPARAR_POLENS);
        }
    }

    private void executarDisparos(int quantidade, Estado proximo) {
        shooter.update();
        if (quantidade <= 0) {
            intakeTransfer.parar();
            shooter.parar();
            mudarEstado(proximo);
            return;
        }

        if (!shooter.isVelocidadeAtingida() || !shooter.isTravaAberta()) {
            intakeTransfer.parar();
            if (tempoEstado.seconds() >= TIMEOUT_TRANSFERENCIA) {
                abortar("RPM caiu antes dos disparos");
            }
            return;
        }

        double periodo = TEMPO_TRANSFERENCIA_POR_POLEN + TEMPO_PAUSA_ENTRE_POLENS;
        double tempo = tempoEstado.seconds();
        int unidadeAtual = (int) Math.floor(tempo / periodo);
        if (unidadeAtual < quantidade) {
            double dentroPeriodo = tempo % periodo;
            if (dentroPeriodo < TEMPO_TRANSFERENCIA_POR_POLEN) {
                intakeTransfer.ligarTransfer(1.0);
            } else {
                intakeTransfer.parar();
            }
            polensAlimentados = Math.min(unidadeAtual, quantidade);
        } else {
            intakeTransfer.parar();
            polensAlimentados = quantidade;
            shooter.parar();
            mudarEstado(proximo);
        }
    }

    private void mudarEstado(Estado novoEstado) {
        estado = novoEstado;
        tempoEstado.reset();
        movimentoConfigurado = false;
        giroIniciado = false;
        if (novoEstado == Estado.ALINHAR_INICIAL || novoEstado == Estado.ALINHAR_FINAL) {
            alinhamentoIniciado = false;
        }
        if (novoEstado == Estado.SHOOTER_INICIAL || novoEstado == Estado.COLETAR_POLENS) {
            presetShooterSelecionado = false;
        }
    }

    private void abortar(String motivo) {
        motivoAbortado = motivo == null || motivo.isEmpty()
                ? "Falha sem motivo informado" : motivo;
        if (repositoryAprilTag != null) repositoryAprilTag.cancelAlignment();
        estado = Estado.ABORTADO;
        tempoEstado.reset();
        pararTudo();
    }

    private void resetarEncoders() {
        motorDir.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEsq.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        voltarParaRunUsingEncoder();
    }

    private void voltarParaRunUsingEncoder() {
        motorDir.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorEsq.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void pararDrive() {
        motorDir.setPower(0.0);
        motorEsq.setPower(0.0);
    }

    private void pararTudo() {
        pararDrive();
        if (intakeTransfer != null) intakeTransfer.parar();
        if (shooter != null) shooter.parar();
    }

    private int centimetrosParaTicks(double distanciaCm) {
        return (int) Math.round(distanciaCm * TICKS_POR_CM);
    }

    private void atualizarTelemetry() {
        telemetry.addData("Estado", estado);
        telemetry.addData("Tempo estado", "%.2f s", tempoEstado.seconds());
        telemetry.addData("Heading", "%.1f graus", gyro == null ? 0.0 :
                gyro.getContinuousAngle());
        telemetry.addData("Polens", "%d / %d", polensAlimentados, polensParaDisparar);
        telemetry.addData("RPM", shooter == null ? "-" : "%.0f", shooter == null ? 0.0 :
                shooter.getVelocidadeRealRpm());
        if (shooter != null) {
            telemetry.addData("RPM estavel", shooter.isVelocidadeAtingida());
            telemetry.addData("Trava", shooter.isTravaAberta() ? "ABERTA" : "FECHADA");
        }
        if (repositoryAprilTag != null) {
            RepositoryAprilTag.DebugSnapshot snapshot = repositoryAprilTag.getDebugSnapshot();
            if (snapshot != null) {
                telemetry.addData("AprilTag", snapshot.alvoEncontrado ? "encontrada" : "procurando");
                telemetry.addData("Tag status", snapshot.status);
                telemetry.addData("Bearing/Range", "%.1f / %.1f", snapshot.bearing,
                        snapshot.range);
                telemetry.addData("Ciclos estaveis", snapshot.ciclosEstaveis);
                telemetry.addData("Correcao heading", "%.3f",
                        repositoryAprilTag.getLastHeadingCorrection());
            }
        }
        if (estado == Estado.ABORTADO) telemetry.addData("Motivo", motivoAbortado);
        telemetry.update();
    }
}