package org.firstinspires.ftc.teamcode.br.sis;

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
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
        public double cutoffDistanciaDetectada = 5;

        //Quanto tempo um artefato deve permanecer como detectado (milissegundos).
        public long duracaoBufferDeteccao = 1000;
    }
    public static Params parametros = new Params();
    
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

    //-- ATRIBUTOS --

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

    private final Timing.Stopwatch cronometroTempoSemArtefato = new Timing.Stopwatch();

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
                    motorEntrada.setPower(parametros.forcaAltaAtividade);
                    motorSaida.setPower(parametros.forcaAltaAtividade);

                    servoApoioPrimario.setPower(parametros.forcaMeiaAtividade);
                    servoApoioSecundario.setPower(parametros.forcaAltaAtividade);
                    break;
                case COLETAR_ARTEFATO:
                    motorEntrada.setPower(parametros.forcaAltaAtividade);
                    if (detectarArtefato()) {
                        motorSaida.setPower(0);
                    } else {
                        motorSaida.setPower(parametros.forcaMeiaAtividade);
                    }

                    servoApoioPrimario.setPower(parametros.forcaMeiaAtividade);
                    servoApoioSecundario.setPower(-parametros.forcaMeiaAtividade);
                    break;
                case EJETAR_ARTEFATO:
                    motorEntrada.setPower(-parametros.forcaAltaAtividade);
                    motorSaida.setPower(-parametros.forcaAltaAtividade);

                    servoApoioPrimario.setPower(-parametros.forcaAltaAtividade);
                    servoApoioSecundario.setPower(-parametros.forcaAltaAtividade);
                    break;
                case SEGURAR_ARTEFATO:
                    motorEntrada.setPower(parametros.forcaBaixaAtividade);
                    motorSaida.setPower(0);

                    servoApoioPrimario.setPower(parametros.forcaMeiaAtividade);
                    servoApoioSecundario.setPower(-parametros.forcaMeiaAtividade);
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

    /**
     *  Retorna um resumo do modo atual do controlador.
     * @return (string)
     */
    public String obterResumoModo() {
        return modo.name();
    }

    //-- COMANDOS --

    /**
     * Ativa o intake num modo solicitado.
     *
     *<p>Volta para modo inicial após o comando ser encerrado.</p>
     *
     * @param modo Modo de ativação desejado
     * @return (novo Comando)
     */
    public Command ativar(Modo modo) {
        return new ComAtivar(modo, this);
    }

    public void adicionarDepuracao(Telemetry telemetria) {
        telemetria.addData(getName().toUpperCase() + " : " + "Artef detectado? (Booleano)" , this::detectarArtefato);
        telemetria.addData(getName().toUpperCase() + " : " + "Modo" , this::obterResumoModo);
    }
}

class ComAtivar extends CommandBase {
    public final Intake subIntake;
    public final Intake.Modo modoDesejado;

    private Intake.Modo modoInicial;

    public ComAtivar(Intake.Modo modoDesejado, Intake subIntake) {
        this.modoDesejado = modoDesejado;

        this.subIntake = subIntake;
        addRequirements(this.subIntake);
    }

    @Override
    public void initialize() {
        modoInicial = subIntake.modo;
    }

    @Override
    public void execute() {
        subIntake.modo = modoDesejado;
    }

    @Override
    public void end(boolean interrupted) {
        subIntake.modo = modoInicial;
    }
}