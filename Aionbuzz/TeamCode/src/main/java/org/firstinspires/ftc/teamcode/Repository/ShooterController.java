package org.firstinspires.ftc.teamcode.Repository;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


public class ShooterController {

    public static class Preset {
        public final String nome;
        public final double rpm;
        public final double anguloCapuzGraus;
        public final double distancia;

        public Preset(String nome, double rpm, double anguloCapuzGraus, double distancia) {
            this.nome = nome;
            this.rpm = rpm;
            this.anguloCapuzGraus = anguloCapuzGraus;
            this.distancia = distancia;
        }
    }

    public static class Config {
        public String nomeMotorShooter = "motorShooter";
        public String nomeServoAngulacao = "servoAngulacao";
        public String nomeServoTrava = "servoTrava";
        public double ticksPorVolta = 28.0;
        public double toleranciaRpm = 50.0;
        public double tempoEstavelSegundos = 0.25;
        public double anguloCapuzZeroGraus = 0.0;
        public double relacaoServoPorCapuz = (19.0 / 16.0);
        public double sentidoCapuz = 1.0;
        public double cursoServoConfiguradoGraus = 300.0;
        public double posicaoServoNoZero = 0.35;
        public double posicaoServoMinima = 0.15;
        public double posicaoServoMaxima = 0.85;
        public DcMotorSimple.Direction direcaoMotorShooter = DcMotorSimple.Direction.REVERSE;
        public Servo.Direction direcaoServoAngulacao = Servo.Direction.REVERSE;
        public double posicaoTravaFechada = 0.0;
        public double posicaoTravaAberta = 0.5;
        public Preset presetB = new Preset("B", 2850.0,
                200.0, 110.0);
        public Preset presetX = new Preset("X", 3000.0,
                170.0, 100.0);
    }

    private final DcMotorEx motorShooter;
    private final Servo servoAngulacao;
    private final Servo servoTrava;
    private final Config config;
    private double anguloCapuzAlvoGraus;
    private double deslocamentoServoCalculadoGraus;
    private double posicaoServoCalculada;
    private boolean angulacaoValida;
    private String diagnosticoAngulacao = "Aguardando inicializacao";
    private double rpmAlvo;
    private double distanciaAlvo;
    private double velocidadeRealRpm;
    private double erroRpm;
    private boolean shooterAtivo;
    private boolean velocidadeAtingida;
    private boolean travaAberta;
    private long inicioEstabilidadeNs;
    private Preset presetAtual;

    public ShooterController(HardwareMap hardwareMap) {
        this(hardwareMap, new Config());
    }

    public ShooterController(HardwareMap hardwareMap, Config config) {
        if (config == null) {
            throw new IllegalArgumentException("A configuracao nao pode ser nula");
        }
        this.config = config;
        anguloCapuzAlvoGraus = config.anguloCapuzZeroGraus;
        posicaoServoCalculada = config.posicaoServoNoZero;
        motorShooter = hardwareMap.get(DcMotorEx.class, config.nomeMotorShooter);
        servoAngulacao = hardwareMap.get(Servo.class, config.nomeServoAngulacao);
        servoTrava = hardwareMap.get(Servo.class, config.nomeServoTrava);
        servoAngulacao.setDirection(config.direcaoServoAngulacao);
        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooter.setDirection(config.direcaoMotorShooter);
        motorShooter.setPower(0.0);
        parar();
    }

    public void start() {
        servoAngulacao.setPosition(config.posicaoServoNoZero);
        posicaoServoCalculada = config.posicaoServoNoZero;
        anguloCapuzAlvoGraus = config.anguloCapuzZeroGraus;
        motorShooter.setVelocity(0.0);
        fecharTrava();
    }

    public void selecionarPresetB() {
        selecionarPreset(config.presetB);
    }

    public void selecionarPresetX() {
        selecionarPreset(config.presetX);
    }

    public void selecionarPreset(Preset preset) {
        if (preset == null) {
            throw new IllegalArgumentException("O preset nao pode ser nulo");
        }
        presetAtual = preset;
        anguloCapuzAlvoGraus = preset.anguloCapuzGraus;
        rpmAlvo = preset.rpm;
        distanciaAlvo = preset.distancia;
        shooterAtivo = true;
        velocidadeAtingida = false;
        inicioEstabilidadeNs = 0L;
        fecharTrava();
    }

    public void parar() {
        shooterAtivo = false;
        rpmAlvo = 0.0;
        velocidadeAtingida = false;
        inicioEstabilidadeNs = 0L;
        motorShooter.setVelocity(0.0);
        fecharTrava();
    }

    public void update() {
        controlarAngulacao();
        controlarShooter();
        controlarTrava();
    }

    public void fecharTrava() {
        travaAberta = false;
        servoTrava.setPosition(config.posicaoTravaFechada);
    }

    private boolean calcularPosicaoServo(double alvoCapuzGraus) {
        angulacaoValida = false;
        if (!Double.isFinite(alvoCapuzGraus)
                || !Double.isFinite(config.relacaoServoPorCapuz)
                || config.relacaoServoPorCapuz <= 0.0
                || !Double.isFinite(config.cursoServoConfiguradoGraus)
                || config.cursoServoConfiguradoGraus <= 0.0
                || !Double.isFinite(config.posicaoServoNoZero)
                || config.posicaoServoNoZero < config.posicaoServoMinima
                || config.posicaoServoNoZero > config.posicaoServoMaxima
                || config.posicaoServoMinima < 0.0
                || config.posicaoServoMaxima > 1.0
                || config.posicaoServoMaxima <= config.posicaoServoMinima) {
            diagnosticoAngulacao = "Configuracao de zero/servo invalida";
            return false;
        }
        double deltaCapuz = alvoCapuzGraus - config.anguloCapuzZeroGraus;
        deslocamentoServoCalculadoGraus = config.sentidoCapuz
                * deltaCapuz / config.relacaoServoPorCapuz;
        double posicao = config.posicaoServoNoZero
                + (deslocamentoServoCalculadoGraus / config.cursoServoConfiguradoGraus)
                * (config.posicaoServoMaxima - config.posicaoServoMinima);
        if (!Double.isFinite(deslocamentoServoCalculadoGraus)
                || !Double.isFinite(posicao)
                || posicao < config.posicaoServoMinima
                || posicao > config.posicaoServoMaxima) {
            diagnosticoAngulacao = "Alvo fora do curso normalizado calibrado";
            return false;
        }
        posicaoServoCalculada = Range.clip(posicao,
                config.posicaoServoMinima, config.posicaoServoMaxima);
        angulacaoValida = true;
        diagnosticoAngulacao = "Alvo valido em relacao ao zero mecanico";
        return true;
    }

    private void controlarAngulacao() {
        if (calcularPosicaoServo(anguloCapuzAlvoGraus)) {
            servoAngulacao.setPosition(posicaoServoCalculada);
        }
    }

    private void controlarShooter() {
        if (!shooterAtivo || !Double.isFinite(rpmAlvo) || rpmAlvo <= 0.0) {
            motorShooter.setVelocity(0.0);
            velocidadeRealRpm = 0.0;
            erroRpm = 0.0;
            velocidadeAtingida = false;
            inicioEstabilidadeNs = 0L;
            return;
        }
        motorShooter.setVelocity(rpmParaTicksPorSegundo(rpmAlvo));
        velocidadeRealRpm = ticksPorSegundoParaRpm(Math.abs(motorShooter.getVelocity()));
        erroRpm = Math.abs(rpmAlvo - velocidadeRealRpm);
        if (Double.isFinite(erroRpm) && erroRpm <= config.toleranciaRpm) {
            if (inicioEstabilidadeNs == 0L) {
                inicioEstabilidadeNs = System.nanoTime();
            }
            double tempo = (System.nanoTime() - inicioEstabilidadeNs) / 1_000_000_000.0;
            velocidadeAtingida = tempo >= config.tempoEstavelSegundos;
        } else {
            inicioEstabilidadeNs = 0L;
            velocidadeAtingida = false;
        }
    }

    private void controlarTrava() {
        travaAberta = shooterAtivo && velocidadeAtingida;
        servoTrava.setPosition(travaAberta
                ? config.posicaoTravaAberta : config.posicaoTravaFechada);
    }

    private double rpmParaTicksPorSegundo(double rpm) {
        return rpm * config.ticksPorVolta / 60.0;
    }

    private double ticksPorSegundoParaRpm(double ticksPorSegundo) {
        return ticksPorSegundo * 60.0 / config.ticksPorVolta;
    }

    public boolean isShooterAtivo() { return shooterAtivo; }
    public boolean isVelocidadeAtingida() { return velocidadeAtingida; }
    public boolean isTravaAberta() { return travaAberta; }
    public boolean isAngulacaoValida() { return angulacaoValida; }
    public String getNomePreset() { return presetAtual == null ? "Nenhum - motor desligado" : presetAtual.nome; }
    public double getRpmAlvo() { return shooterAtivo ? rpmAlvo : 0.0; }
    public double getVelocidadeRealRpm() { return velocidadeRealRpm; }
    public double getErroRpm() { return erroRpm; }
    public double getDistanciaAlvo() { return distanciaAlvo; }
    public double getAnguloCapuzAlvoGraus() { return anguloCapuzAlvoGraus; }
    public double getDeslocamentoServoCalculadoGraus() { return deslocamentoServoCalculadoGraus; }
    public double getPosicaoServoCalculada() { return posicaoServoCalculada; }
    public double getPosicaoServoReportada() { return servoAngulacao.getPosition(); }
    public String getDiagnosticoAngulacao() { return diagnosticoAngulacao; }

    public void stop() {
        parar();
        motorShooter.setPower(0.0);
    }
}