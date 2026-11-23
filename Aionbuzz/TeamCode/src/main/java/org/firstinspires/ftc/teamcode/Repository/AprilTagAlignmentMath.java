package org.firstinspires.ftc.teamcode.Repository;

public class AprilTagAlignmentMath {

    public enum Status {
        PROCURANDO_ALVO,
        APROXIMANDO,
        CENTRALIZADO,
        PRONTO
    }

    public static class Input {
        public final boolean alvoValido;
        public final double bearing;
        public final double range;

        public Input(boolean alvoValido, double bearing, double range) {
            this.alvoValido = alvoValido;
            this.bearing = bearing;
            this.range = range;
        }

        public static Input semAlvo() {
            return new Input(false, 0.0, 0.0);
        }

        public static Input alvo(double bearing, double range) {
            return new Input(true, bearing, range);
        }
    }

    public static class Output {
        public final Status status;
        public final double avanco;
        public final double giro;
        public final double erroBearing;
        public final double erroRange;
        public final int ciclosEstaveis;

        private Output(Status status, double avanco, double giro,
                       double erroBearing, double erroRange,
                       int ciclosEstaveis) {
            this.status = status;
            this.avanco = avanco;
            this.giro = giro;
            this.erroBearing = erroBearing;
            this.erroRange = erroRange;
            this.ciclosEstaveis = ciclosEstaveis;
        }
    }

    private static class PID {
        private final double kp;
        private final double ki;
        private final double kd;
        private final double integralLimite;

        private double integral;
        private double erroAnterior;
        private boolean possuiErroAnterior;

        PID(double kp, double ki, double kd, double integralLimite) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.integralLimite = integralLimite;
        }

        double calculate(double erro, double deltaTime) {
            if (deltaTime <= 0.0 || Double.isNaN(deltaTime)) {
                deltaTime = 0.05;
            }

            integral += erro * deltaTime;
            integral = clip(integral, -integralLimite, integralLimite);

            double derivada = 0.0;
            if (possuiErroAnterior) {
                derivada = (erro - erroAnterior) / deltaTime;
            }

            erroAnterior = erro;
            possuiErroAnterior = true;

            return kp * erro + ki * integral + kd * derivada;
        }

        void reset() {
            integral = 0.0;
            erroAnterior = 0.0;
            possuiErroAnterior = false;
        }
    }

    private final double distanciaAlvo;
    private final double toleranciaBearing;
    private final double toleranciaRange;
    private final double maxGiro;
    private final double maxAvanco;
    private final int ciclosAlvoEstavel;
    private final double variacaoMaximaGiro;
    private final double variacaoMaximaAvanco;

    private final PID pidBearing;
    private final PID pidRange;

    private int ciclosEstaveis;
    private double ultimoGiro;
    private double ultimoAvanco;

    /** Configuração inicial conservadora para o robô. */
    public AprilTagAlignmentMath() {
        this(
                18.0,       // distância desejada, em polegadas
                0.012, 0.0, 0.0015, // PID do bearing
                0.025, 0.0, 0.001,  // PID do range
                2.0,        // tolerância de bearing
                2.0,        // tolerância de distância
                0.15,       // giro máximo
                0.10,       // avanço máximo
                3,          // ciclos estáveis
                0.025,      // variação máxima do giro por ciclo
                0.020       // variação máxima do avanço por ciclo
        );
    }

    public AprilTagAlignmentMath(
            double distanciaAlvo,
            double kpBearing,
            double kiBearing,
            double kdBearing,
            double kpRange,
            double kiRange,
            double kdRange,
            double toleranciaBearing,
            double toleranciaRange,
            double maxGiro,
            double maxAvanco,
            int ciclosAlvoEstavel,
            double variacaoMaximaGiro,
            double variacaoMaximaAvanco) {

        if (ciclosAlvoEstavel <= 0) {
            throw new IllegalArgumentException(
                    "ciclosAlvoEstavel deve ser positivo");
        }
        if (distanciaAlvo <= 0.0 || maxGiro <= 0.0 || maxAvanco <= 0.0) {
            throw new IllegalArgumentException(
                    "distância e potências máximas devem ser positivas");
        }

        this.distanciaAlvo = distanciaAlvo;
        this.toleranciaBearing = toleranciaBearing;
        this.toleranciaRange = toleranciaRange;
        this.maxGiro = maxGiro;
        this.maxAvanco = maxAvanco;
        this.ciclosAlvoEstavel = ciclosAlvoEstavel;
        this.variacaoMaximaGiro = variacaoMaximaGiro;
        this.variacaoMaximaAvanco = variacaoMaximaAvanco;

        // Com Ki = 0, o controlador começa como PD e evita windup.
        this.pidBearing = new PID(
                kpBearing, kiBearing, kdBearing, 20.0);
        this.pidRange = new PID(
                kpRange, kiRange, kdRange, 20.0);
    }

    /** Processa uma leitura usando 50 ms como período padrão. */
    public Output update(Input input) {
        return update(input, 0.05);
    }

    /** Processa uma leitura usando o período real do loop. */
    public Output update(Input input, double deltaTime) {
        if (input == null || !input.alvoValido) {
            reset();
            return output(
                    Status.PROCURANDO_ALVO,
                    0.0,
                    0.0,
                    0.0,
                    0.0
            );
        }

        double erroBearing = input.bearing;
        double erroRange = input.range - distanciaAlvo;

        boolean bearingCorreto =
                Math.abs(erroBearing) <= toleranciaBearing;
        boolean distanciaCorreta =
                Math.abs(erroRange) <= toleranciaRange;

        if (bearingCorreto && distanciaCorreta) {
            ciclosEstaveis++;
            pidBearing.reset();
            pidRange.reset();

            Status status = ciclosEstaveis >= ciclosAlvoEstavel
                    ? Status.PRONTO
                    : Status.CENTRALIZADO;

            ultimoGiro = 0.0;
            ultimoAvanco = 0.0;

            return output(
                    status,
                    0.0,
                    0.0,
                    erroBearing,
                    erroRange
            );
        }

        ciclosEstaveis = 0;

        double giroDesejado = pidBearing.calculate(
                erroBearing, deltaTime);
        double avancoDesejado = pidRange.calculate(
                erroRange, deltaTime);

        giroDesejado = clip(
                giroDesejado, -maxGiro, maxGiro);
        avancoDesejado = clip(
                avancoDesejado, -maxAvanco, maxAvanco);

        double giro = limitarVariacao(
                ultimoGiro,
                giroDesejado,
                variacaoMaximaGiro
        );
        double avanco = limitarVariacao(
                ultimoAvanco,
                avancoDesejado,
                variacaoMaximaAvanco
        );

        ultimoGiro = giro;
        ultimoAvanco = avanco;

        return output(
                Status.APROXIMANDO,
                avanco,
                giro,
                erroBearing,
                erroRange
        );
    }

    public void reset() {
        ciclosEstaveis = 0;
        ultimoGiro = 0.0;
        ultimoAvanco = 0.0;
        pidBearing.reset();
        pidRange.reset();
    }

    public int getCiclosEstaveis() {
        return ciclosEstaveis;
    }

    public double getDistanciaAlvo() {
        return distanciaAlvo;
    }

    private Output output(Status status, double avanco, double giro,
                          double erroBearing, double erroRange) {
        return new Output(
                status,
                avanco,
                giro,
                erroBearing,
                erroRange,
                ciclosEstaveis
        );
    }

    private static double limitarVariacao(
            double atual,
            double desejado,
            double variacaoMaxima) {

        double diferenca = desejado - atual;
        return clip(
                desejado,
                atual - variacaoMaxima,
                atual + variacaoMaxima
        );
    }

    private static double clip(double valor, double minimo, double maximo) {
        return Math.max(minimo, Math.min(maximo, valor));
    }
}
