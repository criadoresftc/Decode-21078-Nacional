package org.firstinspires.ftc.teamcode.br.sistema;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

/**
 *  Implementação de um subsistema para a Torreta Giratória do nosso robô.
 *  <br><br>
 *  Baseado em: <a href="https://docs.wpilib.org/pt/stable/docs/software/advanced-controls/introduction/tuning-turret.html">...</a>
 */
@Configurable
public class Torreta extends SubsystemBase {
    //-- PARÂMETROS --
    public static class Params {
        //Constante de conversão ticks/rotação (rotação = 360º).
        @Sorter(sort = 1)
        public double ticksPorRotacao = 33860.26666666667;

        //Limitação do modelo físico (graus, sentido anti-horário).
        @Sorter(sort = 2)
        public double limitacaoMaxima = 140;
        @Sorter(sort = 3)
        public double limitacaoMinima = 10;

        //Tolerância do controlador (graus)
        @Sorter(sort = 4)
        public double tolerancia = 0.5;

        //Constantes do FeedForward (kS = volts, kV = volts / graus/s).
        @Sorter(sort = 5)
        public double kV = 0.03;
        @Sorter(sort = 6)
        public double kS = 1;

        //Ganhos do PID (apenas o P e D neste caso).
        @Sorter(sort = 7)
        public double ganhoProporcional = 0.2;
        @Sorter(sort = 8)
        public double ganhoDerivado = 0;
    }
    public static Params parametros = new Params();

    //-- ATRIBUTOS --
    /**
     *  Switch ligar/desligar.
     */
    public boolean ativo = true;
    /**
     *  Posição alvo em graus.
     */
    public double posicao;

    /**
     *  Offset inicial em graus.
     */
    public double posicaoInicial;

    //-- ATUADORES E SENSORES --
    private final Motor.Encoder encoder;
    private final Motor.Direction direcaoEncoder;

    private final CRServo servoMotor;
    private final Motor.Direction direcaoMotor;

    private final VoltageSensor sensorEnergia;

    //-- RELATÓRIO --
    //Armazena informações importantes do último passo do controlador.
    private static class RelatorioControle {
        double posicaoAlvo, posicaoMedida, erroMedido, tensaoEletricaRespondida = 0;
        boolean noAlvo = false;
        double momento = 0;

        private void atualizar(double posicaoAlvo, double posicaoMedida, double tensaoEletricaRespondida, double momento) {
            this.posicaoAlvo = posicaoAlvo;
            this.posicaoMedida = posicaoMedida;
            erroMedido = posicaoAlvo - posicaoMedida;
            this.noAlvo = Math.abs(erroMedido) <= parametros.tolerancia;
            this.tensaoEletricaRespondida = tensaoEletricaRespondida;

            this.momento = momento;
        }
    }
    private final RelatorioControle relatorioControle = new RelatorioControle();

    public Torreta(HardwareMap hardwareMap) {
        encoder = new Motor(hardwareMap, "intake").encoder;
        servoMotor = hardwareMap.get(CRServo.class,"torretaServo");

        direcaoEncoder = Motor.Direction.REVERSE;
        direcaoMotor = Motor.Direction.REVERSE;

        encoder.reset();

        sensorEnergia = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void periodic() {
        //-- CONTROLADOR --
        //Aqui é definido como o modelo físico deve reagir diante à mudança dos atributos.
        final double posicaoAtual = MathUtils.normalizeDegrees(posicaoInicial + encoder.getPosition() * direcaoEncoder.getMultiplier() / parametros.ticksPorRotacao * 360, true);
        final double posicaoAlvo = MathUtils.clamp(MathUtils.normalizeDegrees(posicao, true), parametros.limitacaoMinima, parametros.limitacaoMaxima);

        final double erro = posicaoAlvo - posicaoAtual;

        final double momentoAtual = System.nanoTime() / 1E9;

        double tensaoEnviada = 0;
        if(ativo && Math.abs(erro) > parametros.tolerancia) {
            final double periodo = momentoAtual - relatorioControle.momento;

            tensaoEnviada = erro * parametros.ganhoProporcional + (erro - relatorioControle.erroMedido) / periodo * parametros.ganhoDerivado +
                    (posicaoAlvo - relatorioControle.posicaoAlvo) / periodo * parametros.kV;

            //Adiciona kS posteriormente, pois só é possível obter a movimentação instantânea da posição alvo.
            tensaoEnviada += Math.signum(tensaoEnviada) * parametros.kS;
        }

        servoMotor.setPower(tensaoEnviada / sensorEnergia.getVoltage() * direcaoMotor.getMultiplier());

        relatorioControle.atualizar(posicaoAlvo, posicaoAtual, tensaoEnviada, momentoAtual);
    }

    /**
     *  Retorna a última posição real registrada pelo controlador.
     * @return (graus)
     */
    public double obterPosicaoRegistrada() {
        return relatorioControle.posicaoMedida;
    }

    public void adicionarDepuracao(Telemetry telemetria) {
        telemetria.addData(getName().toUpperCase() + " : " + "Pos Alvo (Graus)" , relatorioControle.posicaoAlvo);
        telemetria.addData(getName().toUpperCase() + " : " + "Pos Atual (Graus)" , relatorioControle.posicaoMedida);
        telemetria.addData(getName().toUpperCase() + " : " + "Erro (Graus)", relatorioControle.erroMedido);
        telemetria.addData(getName().toUpperCase() + " : " + "no Alvo? (Booleano)", relatorioControle.noAlvo);
        telemetria.addData(getName().toUpperCase() + " : " + "Saída (Volts)", relatorioControle.tensaoEletricaRespondida);
    }
}
