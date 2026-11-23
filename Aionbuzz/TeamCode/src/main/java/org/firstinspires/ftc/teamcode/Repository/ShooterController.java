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
        public final double anguloCapuz;
        public final double distancia;

        public Preset(String nome, double rpm, double anguloCapuz, double distancia) {
            this.nome = nome;
            this.rpm = rpm;
            this.anguloCapuz = anguloCapuz;
            this.distancia = distancia;
        }
    }

    public static class Config {
        public double ticksPorVolta = 28.0;
        public double toleranciaRpm = 50.0;
        public double tempoEstavelSegundos = 0.25;

        public double dentesServo = 19.0;
        public double dentesCapuz = 16.0;
        public double sentidoEngrenagem = -1.0;
        public double anguloCapuzZero = 10.0;
        public double anguloServoZero = 100.0;
        public double anguloServoMinimo = 0.0;
        public double anguloServoMaximo = 300.0;
        public double posicaoMinima = 0.15;
        public double posicaoMaxima = 0.85;

        public double posicaoTravaFechada = 0.0;
        public double posicaoTravaAberta = 0.5;

        public boolean inverterMotor = true;
        public Preset presetB = new Preset("B", 2850.0, 200.0, 110.0);
        public Preset presetX = new Preset("X", 3000.0, 170.0, 100.0);
    }

    private final DcMotorEx motorShooter;
    private final Servo servoAngulacao;
    private final Servo servoTrava;
    private final Config config;

    private Preset presetAtual;
    private double rpmAlvo;
    private double anguloAlvo;
    private double distanciaAlvo;
    private double velocidadeRealRpm;
    private double erroRpm;
    private double anguloServoCalculado;
    private double posicaoServoCalculada;
    private boolean velocidadeAtingida;
    private boolean anguloValido;
    private boolean travaAberta;
    private long inicioEstabilidadeNanos;

    public ShooterController(HardwareMap hardwareMap) {
        this(hardwareMap, new Config());
    }

    public ShooterController(HardwareMap hardwareMap, Config config) {
        this.config = config;
        motorShooter = hardwareMap.get(DcMotorEx.class, "motorShooter");
        servoAngulacao = hardwareMap.get(Servo.class, "servoAngulacao");
        servoTrava = hardwareMap.get(Servo.class, "servoTrava");

        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (config.inverterMotor) {
            motorShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        motorShooter.setPower(0.0);
        fecharTrava();
        desligar();
    }

    public void selecionarPresetB() {
        selecionarPreset(config.presetB);
    }

    public void selecionarPresetX() {
        selecionarPreset(config.presetX);
    }

    public void selecionarPreset(Preset preset) {
        if (preset == null) {
            throw new IllegalArgumentException("O preset não pode ser nulo");
        }
        presetAtual = preset;
        rpmAlvo = preset.rpm;
        anguloAlvo = preset.anguloCapuz;
        distanciaAlvo = preset.distancia;
    }

    public void desligar() {
        rpmAlvo = 0.0;
        presetAtual = null;
        velocidadeAtingida = false;
        inicioEstabilidadeNanos = 0L;
        motorShooter.setVelocity(0.0);
        fecharTrava();
    }


    public void update() {
        controlarShooter();
        controlarAngulacao();
    }

    public boolean abrirTravaSePronto() {
        if (!velocidadeAtingida) {
            fecharTrava();
            return false;
        }
        travaAberta = true;
        servoTrava.setPosition(config.posicaoTravaAberta);
        return true;
    }

    public void fecharTrava() {
        travaAberta = false;
        servoTrava.setPosition(config.posicaoTravaFechada);
    }

    private void controlarShooter() {
        if (rpmAlvo <= 0.0) {
            motorShooter.setVelocity(0.0);
            velocidadeRealRpm = 0.0;
            erroRpm = 0.0;
            velocidadeAtingida = false;
            inicioEstabilidadeNanos = 0L;
            return;
        }

        motorShooter.setVelocity(rpmParaTicks(rpmAlvo));
        velocidadeRealRpm = ticksParaRpm(Math.abs(motorShooter.getVelocity()));
        erroRpm = Math.abs(rpmAlvo - velocidadeRealRpm);

        if (erroRpm <= config.toleranciaRpm) {
            if (inicioEstabilidadeNanos == 0L) {
                inicioEstabilidadeNanos = System.nanoTime();
            }
            double tempo = (System.nanoTime() - inicioEstabilidadeNanos) / 1_000_000_000.0;
            velocidadeAtingida = tempo >= config.tempoEstavelSegundos;
        } else {
            inicioEstabilidadeNanos = 0L;
            velocidadeAtingida = false;
        }
    }

    private void controlarAngulacao() {
        double novaPosicao = calcularPosicaoServo(anguloAlvo);
        if (anguloValido) {
            posicaoServoCalculada = novaPosicao;
            servoAngulacao.setPosition(posicaoServoCalculada);
        }
    }

    private double calcularPosicaoServo(double anguloCapuz) {
        double relacao = config.dentesServo / config.dentesCapuz;
        double variacaoCapuz = anguloCapuz - config.anguloCapuzZero;
        anguloServoCalculado = config.anguloServoZero
                + config.sentidoEngrenagem * variacaoCapuz / relacao;

        anguloValido = anguloServoCalculado >= config.anguloServoMinimo
                && anguloServoCalculado <= config.anguloServoMaximo;
        if (!anguloValido) {
            return posicaoServoCalculada;
        }

        double proporcao = (anguloServoCalculado - config.anguloServoMinimo)
                / (config.anguloServoMaximo - config.anguloServoMinimo);
        return Range.clip(config.posicaoMinima
                        + proporcao * (config.posicaoMaxima - config.posicaoMinima),
                config.posicaoMinima, config.posicaoMaxima);
    }

    private double rpmParaTicks(double rpm) {
        return rpm * config.ticksPorVolta / 60.0;
    }

    private double ticksParaRpm(double ticks) {
        return ticks * 60.0 / config.ticksPorVolta;
    }

    public boolean isVelocidadeAtingida() { return velocidadeAtingida; }
    public boolean isTravaAberta() { return travaAberta; }
    public boolean isAnguloValido() { return anguloValido; }
    public String getNomePreset() { return presetAtual == null ? "Desligado" : presetAtual.nome; }
    public double getRpmAlvo() { return rpmAlvo; }
    public double getVelocidadeRealRpm() { return velocidadeRealRpm; }
    public double getErroRpm() { return erroRpm; }
    public double getAnguloAlvo() { return anguloAlvo; }
    public double getDistanciaAlvo() { return distanciaAlvo; }
    public double getAnguloServoCalculado() { return anguloServoCalculado; }
    public double getPosicaoServoCalculada() { return posicaoServoCalculada; }

    public void stop() {
        desligar();
        motorShooter.setPower(0.0);
    }
}