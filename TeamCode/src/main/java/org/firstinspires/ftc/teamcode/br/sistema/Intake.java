package org.firstinspires.ftc.teamcode.br.sistema;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

/**
 *  Implementação de um subsistema para o Intake do nosso robô.
 */
@Configurable
public class Intake extends SubsystemBase {
    //-- PARÂMETROS --
    public static class Params {
        //Força enviada para os motores dependendo do seu papel em determinado modo (0.0 min - 1.0 máx).
        public double forcaAltaAtividade = 0.9;
        public double forcaMeiaAtividade = 0.5;
        public double forcaBaixaAtividade = 0.2;


        //Até onde um artefato pode ser detectado pelo sensor de distância (centímetros).
        public double cutoffDistanciaDetectada = 3;

        //Quanto tempo um artefato deve permanecer como detectado (milissegundos).
        public long duracaoBufferDeteccao = 1500;
    }
    public static Params parametros = new Params();

    //-- ATRIBUTOS --
    /**
     *  Switch ligar/desligar.
     */
    public boolean ativo = true;
    /**
     *  Modo do controlador.
     */
    public Modo modo = Modo.NAO_FAZER_NADA;
    public enum Modo {
        /**
         *  Envia o artefato em direção à saída.
         */
        ENVIAR_ARTEFATO,
        /**
         *  Coleta o artefato frente a entrada.
         */
        COLETAR_ARTEFATO,
        /**
         *  Ejeta o artefato armazenado em direção à entrada.
         */
        EJETAR_ARTEFATO,
        /**
         *  Segura o artefato armazenando, impedindo ele de sair.
         */
        SEGURAR_ARTEFATO,
        /**
         *  Não faz nada (literalmente).
         */
        NAO_FAZER_NADA
    }

    //-- ATUADORES E SENSORES --
    private final DcMotorEx motorEntrada;
    private final DcMotorEx motorSaida;
    private final CRServo servoApoioPrimario;
    private final CRServo servoApoioSecundario;

    private final Motor.Direction direcaoMotorEntrada;
    private final Motor.Direction direcaoMotorSaida;
    private final Motor.Direction direcaoServoPrimario;
    private final Motor.Direction direcaoServoSecundario;

    private final DistanceSensor sensorArtefato;

    public Intake(HardwareMap hardwareMap) {
        motorEntrada = hardwareMap.get(DcMotorEx.class, "intake");
        motorSaida = hardwareMap.get(DcMotorEx.class, "out");
        servoApoioPrimario = hardwareMap.get(CRServo.class, "levantador");
        servoApoioSecundario = hardwareMap.get(CRServo.class, "subidor");

        direcaoMotorEntrada = Motor.Direction.FORWARD;
        direcaoMotorSaida = Motor.Direction.REVERSE;
        direcaoServoPrimario = Motor.Direction.FORWARD;
        direcaoServoSecundario = Motor.Direction.FORWARD;

        sensorArtefato = (DistanceSensor) hardwareMap.get(ColorSensor.class, "SCor");
    }

    @Override
    public void periodic() {
        //-- CONTROLADOR --
        //Aqui é definido como o modelo físico deve reagir diante à mudança dos atributos.
        double forcaMotorEntrada = 0;
        double forcaMotorSaida = 0;
        double forcaServoPrimario = 0;
        double forcaServoSecundario = 0;

        if(ativo) {
            switch (modo) {
                case ENVIAR_ARTEFATO:
                    forcaMotorEntrada = parametros.forcaAltaAtividade;
                    forcaMotorSaida = parametros.forcaAltaAtividade;

                    forcaServoPrimario = parametros.forcaMeiaAtividade;
                    forcaServoSecundario = parametros.forcaAltaAtividade;
                    break;
                case COLETAR_ARTEFATO:
                    forcaMotorEntrada = parametros.forcaAltaAtividade;
                    if (detectarArtefato()) {
                        forcaMotorSaida = 0;
                    } else {
                        forcaMotorSaida = parametros.forcaMeiaAtividade;
                    }

                    forcaServoPrimario = parametros.forcaMeiaAtividade;
                    forcaServoSecundario = -parametros.forcaMeiaAtividade;
                    break;
                case EJETAR_ARTEFATO:
                    forcaMotorEntrada = -parametros.forcaAltaAtividade;
                    forcaMotorSaida = -parametros.forcaAltaAtividade;

                    forcaServoPrimario = -parametros.forcaAltaAtividade;
                    forcaServoSecundario = -parametros.forcaAltaAtividade;
                    break;
                case SEGURAR_ARTEFATO:
                    forcaMotorEntrada = parametros.forcaBaixaAtividade;
                    if (detectarArtefato()) {
                        forcaMotorSaida = 0;
                    } else {
                        forcaMotorSaida = parametros.forcaMeiaAtividade;
                    }

                    forcaServoPrimario = parametros.forcaMeiaAtividade;
                    forcaServoSecundario = -parametros.forcaMeiaAtividade;
                    break;
                case NAO_FAZER_NADA:
                    forcaMotorEntrada = 0;
                    forcaMotorSaida = 0;

                    forcaServoPrimario = 0;
                    forcaServoSecundario = 0;
                    break;
            }
        }

        motorEntrada.setPower(forcaMotorEntrada * direcaoMotorEntrada.getMultiplier());
        motorSaida.setPower(forcaMotorSaida * direcaoMotorSaida.getMultiplier());
        servoApoioPrimario.setPower(forcaServoPrimario * direcaoServoPrimario.getMultiplier());
        servoApoioSecundario.setPower(forcaServoSecundario * direcaoServoSecundario.getMultiplier());
    }

    /**
     *  Verifica se há um artefato armazenado internamente.
     * @return (booleano)
     */
    public boolean detectarArtefato() {
        final double distanciaDetectada = sensorArtefato.getDistance(DistanceUnit.CM);
        if(distanciaDetectada <= parametros.cutoffDistanciaDetectada) {
            cronometroTempoSemArtefato.start();
            
            return true;
        }
        
        if(cronometroTempoSemArtefato.isTimerOn()) {
            return cronometroTempoSemArtefato.elapsedTime() <= parametros.duracaoBufferDeteccao;
        }
        
        return false;
    }
    private final Timing.Stopwatch cronometroTempoSemArtefato = new Timing.Stopwatch(TimeUnit.MILLISECONDS);

    public void adicionarDepuracao(Telemetry telemetria) {
        telemetria.addData(getName().toUpperCase() + " : " + "Artef detectado? (Booleano)" , this::detectarArtefato);
        telemetria.addData(getName().toUpperCase() + " : " + "Modo" , modo.name());
    }
}