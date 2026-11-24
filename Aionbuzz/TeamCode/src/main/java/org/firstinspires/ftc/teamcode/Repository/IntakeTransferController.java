package org.firstinspires.ftc.teamcode.Repository;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public class IntakeTransferController {

    public enum Estado {
        PARADO,
        INTAKE,
        TRANSFER,
        INTAKE_E_TRANSFER,
        REVERSO
    }

    private final DcMotor motorIntake;
    private final CRServo servoTransfer;

    private final double potenciaMaximaIntake;
    private final double potenciaMaximaTransfer;

    private Estado estadoAtual = Estado.PARADO;


    public IntakeTransferController(HardwareMap hardwareMap) {
        this(hardwareMap, "motorIntake", "servoTransfer", 1.0, 1.0);
    }

    public IntakeTransferController(
            HardwareMap hardwareMap,
            String nomeMotorIntake,
            String nomeServoTransfer,
            double potenciaMaximaIntake,
            double potenciaMaximaTransfer
    ) {
        motorIntake = hardwareMap.get(DcMotor.class, nomeMotorIntake);
        servoTransfer = hardwareMap.get(CRServo.class, nomeServoTransfer);

        this.potenciaMaximaIntake = limitar(Math.abs(potenciaMaximaIntake));
        this.potenciaMaximaTransfer = limitar(Math.abs(potenciaMaximaTransfer));

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIntake.setPower(0.0);
        servoTransfer.setPower(0.0);
    }

    public void init() {
        parar();
    }

    public void ligarIntake() {
        ligarIntake(1.0);
    }

    public void ligarIntake(double potencia) {
        double potenciaAplicada = limitar(Math.abs(potencia))
                * potenciaMaximaIntake;

        motorIntake.setPower(potenciaAplicada);
        servoTransfer.setPower(0.0);
        estadoAtual = potenciaAplicada == 0.0
                ? Estado.PARADO
                : Estado.INTAKE;
    }

    public void ligarTransfer() {
        ligarTransfer(1.0);
    }

    public void ligarTransfer(double potencia) {
        double potenciaAplicada = -limitar(Math.abs(potencia))
                * potenciaMaximaTransfer;

        motorIntake.setPower(0.0);
        servoTransfer.setPower(potenciaAplicada);
        estadoAtual = potenciaAplicada == 0.0
                ? Estado.PARADO
                : Estado.TRANSFER;
    }

    public void ligar() {
        coletar();
    }

    public void coletar() {
        coletar(0.6, 0.6);
    }

    public void coletar(
            double potenciaIntake,
            double potenciaTransfer
    ) {
        double intakeAplicado = limitar(Math.abs(potenciaIntake))
                * potenciaMaximaIntake;
        double transferAplicado = -limitar(Math.abs(potenciaTransfer))
                * potenciaMaximaTransfer;

        motorIntake.setPower(intakeAplicado);
        servoTransfer.setPower(transferAplicado);

        estadoAtual = intakeAplicado == 0.0 && transferAplicado == 0.0
                ? Estado.PARADO
                : Estado.INTAKE_E_TRANSFER;
    }

        public void reverter() {
        reverter(1.0, 1.0);
    }

    public void reverter(
            double potenciaIntake,
            double potenciaTransfer
    ) {
        double intakeAplicado = -limitar(Math.abs(potenciaIntake))
                * potenciaMaximaIntake;
        double transferAplicado = limitar(Math.abs(potenciaTransfer))
                * potenciaMaximaTransfer;

        motorIntake.setPower(intakeAplicado);
        servoTransfer.setPower(transferAplicado);
        estadoAtual = Estado.REVERSO;
    }

        public void parar() {
        motorIntake.setPower(0.0);
        servoTransfer.setPower(0.0);
        estadoAtual = Estado.PARADO;
    }

    public void stop() {
        parar();
    }

    public Estado getEstado() {
        return estadoAtual;
    }

    public double getPotenciaIntake() {
        return motorIntake.getPower();
    }

    public double getPotenciaTransfer() {
        return servoTransfer.getPower();
    }

    private double limitar(double valor) {
        return Range.clip(valor, -1.0, 1.0);
    }
}