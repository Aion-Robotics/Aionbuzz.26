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
        public final double erroRange;
        public final int ciclosEstaveis;

        private Output(Status status, double avanco, double giro,
                       double erroRange, int ciclosEstaveis) {
            this.status = status;
            this.avanco = avanco;
            this.giro = giro;
            this.erroRange = erroRange;
            this.ciclosEstaveis = ciclosEstaveis;
        }
    }

    private final double distanciaAlvo;
    private final double kpBearing;
    private final double kpRange;
    private final double toleranciaBearing;
    private final double toleranciaRange;
    private final double maxGiro;
    private final double maxAvanco;
    private final int ciclosAlvoEstavel;

    private int ciclosEstaveis;

    public AprilTagAlignmentMath() {
        this(18.0, 0.015, 0.025, 2.0, 2.0, 0.15, 0.10, 3);
    }

    public AprilTagAlignmentMath(
            double distanciaAlvo,
            double kpBearing,
            double kpRange,
            double toleranciaBearing,
            double toleranciaRange,
            double maxGiro,
            double maxAvanco,
            int ciclosAlvoEstavel) {
        if (ciclosAlvoEstavel <= 0) {
            throw new IllegalArgumentException("ciclosAlvoEstavel deve ser positivo");
        }

        this.distanciaAlvo = distanciaAlvo;
        this.kpBearing = kpBearing;
        this.kpRange = kpRange;
        this.toleranciaBearing = toleranciaBearing;
        this.toleranciaRange = toleranciaRange;
        this.maxGiro = maxGiro;
        this.maxAvanco = maxAvanco;
        this.ciclosAlvoEstavel = ciclosAlvoEstavel;
    }


    public void reset() {
        ciclosEstaveis = 0;
    }

    public Output update(Input input) {
        if (input == null || !input.alvoValido) {
            reset();
            return output(Status.PROCURANDO_ALVO, 0.0, 0.0, 0.0);
        }

        double erroRange = input.range - distanciaAlvo;
        boolean bearingCorreto = Math.abs(input.bearing) <= toleranciaBearing;
        boolean distanciaCorreta = Math.abs(erroRange) <= toleranciaRange;

        if (bearingCorreto && distanciaCorreta) {
            ciclosEstaveis++;

            Status status = ciclosEstaveis >= ciclosAlvoEstavel
                    ? Status.PRONTO
                    : Status.CENTRALIZADO;

            return output(status, 0.0, 0.0, erroRange);
        }

        ciclosEstaveis = 0;

        double giro = clip(input.bearing * kpBearing, -maxGiro, maxGiro);
        double avanco = erroRange * kpRange;

        // Mantém uma potência mínima quando ainda há erro de distância,
        // evitando que o robô fique parado por causa de um erro muito pequeno.
        if (Math.abs(erroRange) > toleranciaRange && Math.abs(avanco) < 0.08) {
            avanco = Math.copySign(0.08, avanco);
        }

        avanco = clip(avanco, -maxAvanco, maxAvanco);

        return output(Status.APROXIMANDO, avanco, giro, erroRange);
    }

    public int getCiclosEstaveis() {
        return ciclosEstaveis;
    }

    public double getDistanciaAlvo() {
        return distanciaAlvo;
    }

    private Output output(Status status, double avanco, double giro, double erroRange) {
        return new Output(status, avanco, giro, erroRange, ciclosEstaveis);
    }

    private static double clip(double valor, double minimo, double maximo) {
        return Math.max(minimo, Math.min(maximo, valor));
    }
}
