package org.firstinspires.ftc.teamcode.br;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//TODO: Testar no robô + Adicionar JDocs
@Configurable
public class Intake extends SubsystemBase {
    //Expõem as constantes utilizadas globalmente (para Dashboard).
    public static iConstantes constantes = new iConstantes();
    
    public enum Modo {
        ENVIAR_ARTEFATO, COLETAR_ARTEFATO, EJETAR_ARTEFATO, SEGURAR_ARTEFATO, NAO_FAZER_NADA
    }

    /**
     *  Switch ligar/desligar.
     */
    public boolean ativo = true;
    /**
     *  Modo do controlador.
     */
    public Modo modo;

    private final DcMotorEx motorEntrada;
    private final DcMotorEx motorSaida;
    private final CRServo servoApoioPrimario;
    private final CRServo servoApoioSecundario;

    private final DistanceSensor sensorArtefato;

    private final Timing.Stopwatch cronometroTempoSemArtefatoDetectado = new Timing.Stopwatch();

    public Intake(Modo modoInicial, HardwareMap hardwareMap) {
        motorEntrada = hardwareMap.get(DcMotorEx.class, "intake");
        motorSaida = hardwareMap.get(DcMotorEx.class, "out");
        servoApoioPrimario = hardwareMap.get(CRServo.class, "levantador");
        servoApoioSecundario = hardwareMap.get(CRServo.class, "subidor");

        motorSaida.setDirection(DcMotorSimple.Direction.REVERSE);

        sensorArtefato = (DistanceSensor) hardwareMap.get(ColorSensor.class, "SCor");

        modo = modoInicial;
    }

    @Override
    public void periodic() {
        //-- CONTROLADOR --
        //Aqui é definido como o modelo físico deve reagir diante à mudança dos atributos.
        if(ativo) {
            switch (modo) {
                case ENVIAR_ARTEFATO:
                    motorEntrada.setPower(0.8);
                    motorSaida.setPower(0.9);

                    servoApoioPrimario.setPower(0.8);
                    servoApoioSecundario.setPower(0.8);
                    break;
                case COLETAR_ARTEFATO:
                    motorEntrada.setPower(0.9);
                    if (detectarArtefato()) {
                        motorSaida.setPower(0);
                    } else {
                        motorSaida.setPower(0.5);
                    }

                    servoApoioPrimario.setPower(0.5);
                    servoApoioSecundario.setPower(-0.5);
                    break;
                case EJETAR_ARTEFATO:
                    motorEntrada.setPower(-0.9);
                    motorSaida.setPower(-0.9);

                    servoApoioPrimario.setPower(0);
                    servoApoioSecundario.setPower(0);
                    break;
                case SEGURAR_ARTEFATO:
                    motorEntrada.setPower(0.3);
                    motorSaida.setPower(0);

                    servoApoioPrimario.setPower(0);
                    servoApoioSecundario.setPower(0);
                    break;
                case NAO_FAZER_NADA:
                    motorEntrada.setPower(0);
                    motorSaida.setPower(0);

                    servoApoioPrimario.setPower(0);
                    servoApoioSecundario.setPower(0);
                    break;
            }
        }
    }

    public boolean detectarArtefato() {
        final double distanciaDetectada = sensorArtefato.getDistance(DistanceUnit.CM);
        if(distanciaDetectada <= iConstantes.cutoffDistanciaDetectada) {
            cronometroTempoSemArtefatoDetectado.start();
            
            return true;
        }
        
        if(cronometroTempoSemArtefatoDetectado.isTimerOn()) {
            return cronometroTempoSemArtefatoDetectado.elapsedTime() <= iConstantes.duracaoBufferDeteccao;
        }
        
        return false;
    }

    public String obterResumoModo() {
        return modo.name();
    }

    public void adicionarDepuracao(Telemetry telemetria) {
        telemetria.addData(getName().toUpperCase() + " : " + "Artef detectado? (Booleano)" , this::detectarArtefato);
        telemetria.addData(getName().toUpperCase() + " : " + "Modo" , this::obterResumoModo);
    }
}

class iConstantes {
    //Até onde um artefato pode ser detectado pelo sensor de distância (centímetros)
    public static double cutoffDistanciaDetectada = 5;

    //Quanto tempo um artefato deve permanecer como detectado (milissegundos)
    public static long duracaoBufferDeteccao = 1000;
}
