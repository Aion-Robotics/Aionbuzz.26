    package org.firstinspires.ftc.teamcode.Mechanisms;

    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;

    @TeleOp
    public class Shooter3 extends OpMode {

        private DcMotorEx motorShooter;

        private static final String MOTOR_NAME = "motorShooter";
        private static final double MAX_RPM = 5990.4; // RPM do motor modificado para o shooter;
        //Valor a ser mais testado experimentalmente, por conta de modificações no motor

        public void init(){
            motorShooter = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

            motorShooter.setDirection(DcMotorSimple.Direction.FORWARD); // direção do motor
            motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // freio do motor
            motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // zera posição atual do encoder
            motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // motor em modo de leitura de encoder e controle de velocidade

        }
        @Override
        public void loop(){
            double trigger = gamepad1.right_trigger;

            if (trigger < 0.05) {
                trigger = 0.0;
            } // zona morta do trigger

            double ticksPerRevolution = motorShooter.getMotorType().getTicksPerRev();

            double targetRpm = trigger * MAX_RPM; /* calculo do RPM desejado;
             transforma o valor do trigger em potencia do motor de forma linear */
            double targetTicksPerSecond = targetRpm / 60.0 * ticksPerRevolution;
            /* conversão do setVelocity() para RPM (formula = ticks/s = RPM ÷ 60 × ticks/revolução);
             divisão por 60 porque RPM significa "rotações por minuto", mas o SDK precisa de rotações por segundo */
             /*Formula = rotações por segundo = 1000 ÷ 60 = 16,67
             ticks/s = 16,67 × 2781,5 ≈ 46358 ticks/s*/

            if (trigger > 0.0) {
                motorShooter.setVelocity(targetTicksPerSecond);
            } else { motorShooter.setVelocity(0.0);}

            double currentTicksPerSecond = motorShooter.getVelocity(); /* retorna ticks por segundo, é necessário converter para RPM
            Para converter em RPM: currentRpm = currentTicksPerSecond * 60.0 / ticksPerRevolution
            Formula Inversa: RPM = ticks/s × 60 ÷ ticks/revolução */

            double currentRpm = 0.0;

            if (ticksPerRevolution > 0.0) {
                currentRpm = currentTicksPerSecond * 60.0 / ticksPerRevolution;
            }

            telemetry.addData("Trigger", "%.2f", trigger);
            telemetry.addData("RPM Alvo", "%.1f", targetRpm);
            telemetry.addData("RPM Atual", "%.1f", currentRpm);
            telemetry.addData("Ticks/s Atual", "%.1f", currentTicksPerSecond);
            telemetry.addData("Ticks/rev", "%.1f", ticksPerRevolution);
            telemetry.addData("Posição Encoder", motorShooter.getCurrentPosition());
            telemetry.addData("Potência Aplicada", "%.2f", motorShooter.getPower());
            telemetry.update();
        }

        @Override
        public void stop() {
            if (motorShooter != null) {
                motorShooter.setVelocity(0.0);
                motorShooter.setPower(0.0);
            }
        }
    }
