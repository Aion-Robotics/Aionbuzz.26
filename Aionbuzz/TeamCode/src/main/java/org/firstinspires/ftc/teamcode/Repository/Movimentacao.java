package org.firstinspires.ftc.teamcode.Repository;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public class Movimentacao {

    public static final double POTENCIA_MAXIMA_PADRAO = 0.5;

    // Valores da base original: motor com encoder de 537,6 ticks/volta e roda de 9,66 cm.
    private static final double TICKS_POR_VOLTA_PADRAO = 537.6;
    private static final double DIAMETRO_RODA_CM_PADRAO = 9.66;

    private final DcMotor motorDireito;
    private final DcMotor motorEsquerdo;
    private final double ticksPorVolta;
    private final double diametroRodaCm;

    public Movimentacao(HardwareMap hardwareMap) {
        this(
                hardwareMap,
                "motorDir",
                "motorEsq",
                TICKS_POR_VOLTA_PADRAO,
                DIAMETRO_RODA_CM_PADRAO
        );
    }

    public Movimentacao(
            HardwareMap hardwareMap,
            String nomeMotorDireito,
            String nomeMotorEsquerdo,
            double ticksPorVolta,
            double diametroRodaCm
    ) {
        if (ticksPorVolta <= 0 || diametroRodaCm <= 0) {
            throw new IllegalArgumentException("As medidas da base devem ser maiores que zero.");
        }

        this.motorDireito = hardwareMap.get(DcMotor.class, nomeMotorDireito);
        this.motorEsquerdo = hardwareMap.get(DcMotor.class, nomeMotorEsquerdo);
        this.ticksPorVolta = ticksPorVolta;
        this.diametroRodaCm = diametroRodaCm;

        configurarHardware();
    }

    private void configurarHardware() {
        motorEsquerdo.setDirection(DcMotor.Direction.REVERSE);

        motorDireito.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorEsquerdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorDireito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorEsquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        parar();
    }

    public void dirigirArcade(double eixoY, double eixoX, double limite) {
        limite = Range.clip(Math.abs(limite), 0.0, 1.0);

        double frente = Range.clip(eixoY, -1.0, 1.0);
        double curva = Range.clip(eixoX, -1.0, 1.0);

        double potenciaEsquerda = Range.clip(frente - curva, -limite, limite);
        double potenciaDireita = Range.clip(frente + curva, -limite, limite);

        definirPotencias(potenciaEsquerda, potenciaDireita);
    }

    public void dirigirArcade(double eixoY, double eixoX) {
        dirigirArcade(eixoY, eixoX, POTENCIA_MAXIMA_PADRAO);
    }

    public void dirigirTank(double potenciaEsquerda, double potenciaDireita) {
        definirPotencias(potenciaEsquerda, potenciaDireita);
    }

    public void definirPotencias(double potenciaEsquerda, double potenciaDireita) {
        potenciaEsquerda = Range.clip(potenciaEsquerda, -1.0, 1.0);
        potenciaDireita = Range.clip(potenciaDireita, -1.0, 1.0);

        motorEsquerdo.setPower(potenciaEsquerda);
        motorDireito.setPower(potenciaDireita);

        boolean baseParada = potenciaEsquerda == 0.0 && potenciaDireita == 0.0;
        DcMotor.ZeroPowerBehavior comportamento = baseParada
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT;

        motorEsquerdo.setZeroPowerBehavior(comportamento);
        motorDireito.setZeroPowerBehavior(comportamento);
    }

    public void parar() {
        definirPotencias(0.0, 0.0);
    }

    public void resetarEncoders() {
        motorDireito.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEsquerdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDireito.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorEsquerdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moverPorCentimetros(double distanciaCm, double potencia) {
        int deltaTicks = centimetrosParaTicks(distanciaCm);

        int alvoDireito = motorDireito.getCurrentPosition() + deltaTicks;
        int alvoEsquerdo = motorEsquerdo.getCurrentPosition() + deltaTicks;

        motorDireito.setTargetPosition(alvoDireito);
        motorEsquerdo.setTargetPosition(alvoEsquerdo);

        motorDireito.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorEsquerdo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double potenciaLimitada = Range.clip(Math.abs(potencia), 0.0, 1.0);
        motorDireito.setPower(potenciaLimitada);
        motorEsquerdo.setPower(potenciaLimitada);
    }


    public boolean estaOcupada() {
        return motorDireito.isBusy() || motorEsquerdo.isBusy();
    }


    public void voltarParaControleManual() {
        parar();
        motorDireito.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorEsquerdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getDistanciaMediaCm() {
        double distanciaDireita = ticksParaCentimetros(motorDireito.getCurrentPosition());
        double distanciaEsquerda = ticksParaCentimetros(motorEsquerdo.getCurrentPosition());
        return (distanciaDireita + distanciaEsquerda) / 2.0;
    }

    public int getTicksDireito() {
        return motorDireito.getCurrentPosition();
    }

    public int getTicksEsquerdo() {
        return motorEsquerdo.getCurrentPosition();
    }

    private int centimetrosParaTicks(double distanciaCm) {
        return (int) Math.round((distanciaCm / (diametroRodaCm * Math.PI)) * ticksPorVolta);
    }

    private double ticksParaCentimetros(int ticks) {
        return (ticks / ticksPorVolta) * (diametroRodaCm * Math.PI);
    }
}
