package org.firstinspires.ftc.teamcode.br;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
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

    private static class RelatorioControle {
        //Armazena informações importantes de um passo do controlador.
        double posicaoAlvo, posicaoMedida, erroMedido, tensaoEletricaRespondida = 0;
        boolean noAlvo;

        double momento = 0;

        private void atualizar(double posicaoAlvo, double posicaoMedida, double tensaoEletricaRespondida, double momento) {
            this.posicaoAlvo = posicaoAlvo;
            this.posicaoMedida = posicaoMedida;
            erroMedido = posicaoAlvo - posicaoMedida;
            noAlvo = Math.abs(erroMedido) < parametros.tolerancia;
            this.tensaoEletricaRespondida = tensaoEletricaRespondida;

            this.momento = momento;
        }
    }

    //-- ATRIBUTOS --

    /**
     *  Switch ligar/desligar.
     */
    public boolean ativo = true;
    /**
     *  Posição alvo em (Graus).
     */
    public double posicao;

    public final double posicaoInicial;

    public final Motor.Encoder encoder;
    private final com.qualcomm.robotcore.hardware.CRServo servoMotor;

    private final VoltageSensor sensorEnergia;

    private final RelatorioControle ultimoRelatorio = new RelatorioControle();
    public Torreta(double posicaoInicial, HardwareMap hardwareMap) {
        encoder = new Motor(hardwareMap, "intake").encoder;
        servoMotor = hardwareMap.get(CRServo.class, "torretaServo");

        encoder.setDirection(Motor.Direction.REVERSE);
        servoMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder.reset();

        sensorEnergia = hardwareMap.voltageSensor.iterator().next();

        this.posicaoInicial = posicaoInicial;
        this.posicao = this.posicaoInicial;
    }

    @Override
    public void periodic() {
        //-- CONTROLADOR --
        //Aqui é definido como o modelo físico deve reagir diante à mudança dos atributos.

        final double posicaoAtual = MathUtils.normalizeDegrees(posicaoInicial + encoder.getPosition() / parametros.ticksPorRotacao * 360, true);
        final double posicaoAlvo = MathUtils.clamp(MathUtils.normalizeDegrees(posicao, true), parametros.limitacaoMinima, parametros.limitacaoMaxima);

        final double erro = posicaoAlvo - posicaoAtual;

        final double momentoAtual = System.nanoTime() / 1E9;

        double tensaoEnviada = 0;
        if(ativo && Math.abs(erro) > parametros.tolerancia) {
            final double periodo = momentoAtual - ultimoRelatorio.momento;

            tensaoEnviada = erro * parametros.ganhoProporcional + (erro - ultimoRelatorio.erroMedido) / periodo * parametros.ganhoDerivado +
                    (posicaoAlvo - ultimoRelatorio.posicaoAlvo) / periodo * parametros.kV;

            //Adiciona kS posteriormente, pois só é possível obter a movimentação instantânea da posição alvo.
            tensaoEnviada += Math.signum(tensaoEnviada) * parametros.kS;
        }

        servoMotor.setPower(tensaoEnviada / sensorEnergia.getVoltage());

        ultimoRelatorio.atualizar(posicaoAlvo, posicaoAtual, tensaoEnviada, momentoAtual);
    }

    /**
     *  Retorna a posição alvo utilizada pelo controlador no último passo.
     * @return (graus)
     */
    public double obterPosicaoAlvo() {
        return ultimoRelatorio.posicaoAlvo;
    }
    /**
     *  Retorna a posição atual medida pelo controlador no último passo.
     * @return (graus)
     */
    public double obterPosicaoAtual() {
        return ultimoRelatorio.posicaoMedida;
    }
    /**
     *  Retorna o erro de posição medido pelo controlador no último passo.
     * @return (graus)
     */
    public double obterErroPosicao() {
        return ultimoRelatorio.erroMedido;
    }
    /**
     *  Retorna se o controlador chegou na posição alvo no último passo.
     * @return (booleano)
     */
    public boolean estaNaPosicaoAlvo() {
        return ultimoRelatorio.noAlvo;
    }
    /**
     *  Retorna a tensão elétrica enviada pelo controlador no último passo.
     *  @return (volts)
     */
    public double obterSaida() {
        return ultimoRelatorio.tensaoEletricaRespondida;
    }

    //-- COMANDOS --

    /**
     * Rotaciona a torreta até um ângulo desejado.
     *
     * @param angulo Ângulo desejado em (Graus)
     * @return (novo Comando)
     */
    public Command rotacionarAte(double angulo) {
        return new ComRotacionarAte(angulo, this);
    }
    /**
     * Rotaciona a torreta em direção a um alvo.
     *
     * @param supridorDirecaoAlvo Supridor da direção do alvo em (Graus)
     * @return (novo Comando)
     */
    public Command seguirAlvo(DoubleSupplier supridorDirecaoAlvo) {
        return new ComSeguirAlvo(supridorDirecaoAlvo, this);
    }
    /**
     * Rotaciona a torreta para um angulo desejado e permanece até o comando ser interrompido.
     *
     * @param angulo Ângulo desejado em (Graus)
     * @return (novo Comando)
     */
    public Command travarNaPosicao(double angulo) {
        return new ComTravarNaPosicao(angulo, this);
    }

    public void adicionarDepuracao(Telemetry telemetria) {
        telemetria.addData(getName().toUpperCase() + " : " + "Pos Alvo (Graus)" , this::obterPosicaoAlvo);
        telemetria.addData(getName().toUpperCase() + " : " + "Pos Atual (Graus)" , this::obterPosicaoAtual);
        telemetria.addData(getName().toUpperCase() + " : " + "no Alvo? (Booleano)", this::estaNaPosicaoAlvo);
        telemetria.addData(getName().toUpperCase() + " : " + "Erro (Graus)", this::obterErroPosicao);
        telemetria.addData(getName().toUpperCase() + " : " + "Saída (Volts)", this::obterSaida);
    }
}

class ComRotacionarAte extends CommandBase {
    public final Torreta subTorreta;
    public final double posicaoDesejada;

    public ComRotacionarAte(double posicaoDesejada, Torreta subTorreta) {
        this.posicaoDesejada = posicaoDesejada;

        this.subTorreta = subTorreta;
        addRequirements(this.subTorreta);
    }

    @Override
    public void execute() {
        subTorreta.posicao = posicaoDesejada;
    }

    @Override
    public boolean isFinished() {
        return subTorreta.estaNaPosicaoAlvo();
    }
}

class ComSeguirAlvo extends CommandBase {
    public final Torreta subTorreta;
    public final DoubleSupplier supridorPosicaoAlvo;

    public ComSeguirAlvo(DoubleSupplier supridorPosicaoAlvo, Torreta subTorreta) {
        this.supridorPosicaoAlvo = supridorPosicaoAlvo;

        this.subTorreta = subTorreta;
        addRequirements(this.subTorreta);
    }

    @Override
    public void execute() {
        subTorreta.posicao = supridorPosicaoAlvo.getAsDouble();
    }
}

class ComTravarNaPosicao extends CommandBase {
    public final Torreta subTorreta;
    public final double posicaoDesejada;

    public ComTravarNaPosicao(double posicaoDesejada, Torreta subTorreta) {
        this.posicaoDesejada = posicaoDesejada;

        this.subTorreta = subTorreta;
        addRequirements(subTorreta);
    }

    @Override
    public void execute() {
        subTorreta.posicao = posicaoDesejada;
    }
}

