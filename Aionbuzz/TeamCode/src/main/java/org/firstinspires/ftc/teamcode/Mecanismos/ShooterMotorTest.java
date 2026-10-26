package org.firstinspires.ftc.teamcode.Mecanismos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooter Motor Test")
public class ShooterMotorTest extends OpMode {
    private DcMotorEx shooter;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "motorShooter");
        shooter.setDirection(DcMotor.Direction.FORWARD); // direção do motor
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Coloca o motor em modo de funcionamento utilizando o encoder
        //Encoder é utilizado para medir posição e velocidade do motor
    }

    @Override
    public void loop() {
        double power = gamepad1.right_bumper ? 0.20 : 0.0; /* verifica se o botão está pressionado
        Modo de funcionamento do operador:  condição ? valor_se_verdadeiro : valor_se_falso
        Se o right_bumper estiver pressionado: power = 0.20
        Se o botão não estiver pressionado: power = 0.0 */
        shooter.setPower(power);

        telemetry.addData("Potência", power);
        telemetry.addData("Velocidade (ticks/s)", shooter.getVelocity());
        telemetry.update();
    }
}
