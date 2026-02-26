package org.firstinspires.ftc.teamcode.br;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public enum Tarefa {
        ENVIAR_ARTEFATO, COLETAR_ARTEFATO, EJETAR_ARTEFATO, SEGURAR_ARTEFATO, FICAR_PARADO
    }

    public Tarefa tarefa;

    private final DcMotorEx motorEntrada;
    private final DcMotorEx motorSaida;

    private final CRServo servoApoioPrimario;
    private final CRServo servoApoioSecundario;

    private final DistanceSensor sensorArtefato;

    public Intake(Tarefa tarefaInicial, HardwareMap hardwareMap) {
        motorEntrada = hardwareMap.get(DcMotorEx.class, "");
        motorSaida = hardwareMap.get(DcMotorEx.class, "");
        servoApoioPrimario = hardwareMap.get(CRServo.class, "");
        servoApoioSecundario = hardwareMap.get(CRServo.class, "");

        sensorArtefato = (DistanceSensor) hardwareMap.get(ColorSensor.class, "");
    }

    @Override
    public void periodic() {
        final boolean artefatoDetectado;

        switch (tarefa) {
            case ENVIAR_ARTEFATO:
                motorEntrada.setPower(0.5);
                motorSaida.setPower(0.9);

                servoApoioPrimario.setPower(0.8);
                servoApoioSecundario.setPower(0.8);
            break;
            case COLETAR_ARTEFATO:
                motorEntrada.setPower(0.9);
                if(artefatoDetectado) {
                    motorSaida.setPower(0);
                } else {
                    motorSaida.setPower(0.5);
                }

                servoApoioPrimario.setPower(0);
                servoApoioSecundario.setPower(0);
            break;
            case EJETAR_ARTEFATO:
                motorEntrada.setPower(-1);
                motorSaida.setPower(-1);

                servoApoioPrimario.setPower(0);
                servoApoioSecundario.setPower(0);
            break;
            case SEGURAR_ARTEFATO:
                motorEntrada.setPower(0.3);
                motorSaida.setPower(0);

                servoApoioPrimario.setPower(0);
                servoApoioSecundario.setPower(0);
            break;
            case FICAR_PARADO:
                motorEntrada.setPower(0);
                motorSaida.setPower(0);

                servoApoioPrimario.setPower(0);
                servoApoioSecundario.setPower(0);
        }
    }
}

class iConstantes {
    //Até onde um artefato pode ser detectado pelo sensor de distância (centímetros)
    public static double cutoffDistanciaDetectada = 5;
}
