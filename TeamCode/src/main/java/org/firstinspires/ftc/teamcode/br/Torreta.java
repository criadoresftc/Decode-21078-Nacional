package org.firstinspires.ftc.teamcode.br;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.trajectory.constraint.DifferentialDriveKinematicsConstraint;


@Configurable
public final class Torreta extends SubsystemBase {
    private class RelatorioControle {
        //Armazena informações importantes de um passo do controlador.
        double posicaoAlvo, posicaoMedida, erroMedido, tensaoEletricaRespondida = 0;
        boolean noAlvo;

        double momento = 0;

        private void atualizar(double posicaoAlvo, double posicaoMedida, double tensaoEletricaRespondida) {
            momento = System.nanoTime() / 1E9;

            this.posicaoAlvo = posicaoAlvo;
            this.posicaoMedida = posicaoMedida;
            erroMedido = posicaoAlvo - posicaoMedida;
            noAlvo = Math.abs(erroMedido) < tConstantes.tolerancia;
            this.tensaoEletricaRespondida = tensaoEletricaRespondida;
        }
    }

    public boolean ativo = true;

    public final double posicaoInicial;

    private double posicao = 0;

    public final DcMotorEx encoder;
    private final com.qualcomm.robotcore.hardware.CRServo servoMotor;

    private final VoltageSensor sensorEnergia;

    private final RelatorioControle relatorio = new RelatorioControle();
    public Torreta(double posicaoInicial, HardwareMap hardwareMap) {
        encoder = hardwareMap.get(DcMotorEx.class, "intake");
        servoMotor = hardwareMap.get(CRServo.class, "torreta");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sensorEnergia = hardwareMap.voltageSensor.iterator().next();

        this.posicaoInicial = posicaoInicial;
    }

    @Override
    public void periodic() {
        //-- CONTROLADOR --
        //Aqui é definido como o modelo físico deve reagir diante à mudança dos atributos.
        double periodo = System.nanoTime() / 1E9 - relatorio.momento;

        double posicaoAtual = posicaoInicial + encoder.getCurrentPosition() / tConstantes.ticksPorRotacao * 360;
        double posicaoAlvo = posicao;

        double erro = (posicaoAtual - posicaoAlvo);

        double velocidadeErro = 0;
        if(periodo > 1E6) {
            velocidadeErro = (relatorio.erroMedido - erro) / periodo;
        }

        double tensaoEnviada = 0;
        if(ativo &&
                posicao <= tConstantes.limitacaoMaxima &&
                posicao >= tConstantes.limitacaoMinima &&
                Math.abs(erro) > tConstantes.tolerancia
        ) {
            tensaoEnviada = erro * tConstantes.ganhoProporcional + velocidadeErro * tConstantes.ganhoDerivado;

            tensaoEnviada += Math.signum(tensaoEnviada) * tConstantes.kS;
        }

        servoMotor.setPower(tensaoEnviada / sensorEnergia.getVoltage());

        relatorio.atualizar(posicaoAlvo, posicaoAtual, tensaoEnviada);
    }

    public void moverPara(double angulo) {
        posicao = angulo % 360;

        //Inverte a posição após um limite fixo de 200 e -200.
        if(posicao > ((270 + 20 + posicaoInicial) % 360)) {
            posicao -= 360;
        }
        else if(posicao < ((-90 - 20 - posicaoInicial) % 360)) {
            posicao += 360;
        }

        //Realiza um pré-controle, enviando rapidamente um "chute" em direção a posição alvo.
        double posicaoAtual = posicaoInicial + encoder.getCurrentPosition() / tConstantes.ticksPorRotacao * 360;
        double posicaoAlvo = posicao;
        double erro = posicaoAlvo - posicaoAtual;

        double tensaoEnviada = 0;
        if(ativo &&
                posicao <= tConstantes.limitacaoMaxima &&
                posicao >= tConstantes.limitacaoMinima &&
                Math.abs(erro) > tConstantes.tolerancia
        ) {
            tensaoEnviada = erro * tConstantes.kV;
        }
        servoMotor.setPower(tensaoEnviada / sensorEnergia.getVoltage());

        relatorio.atualizar(posicaoAlvo, posicaoAtual, tensaoEnviada);
    }

    /**
     *  Retorna a posição alvo utilizada pelo controlador a cada passo.
     * @return (graus)
     */
    public double obterPosicaoAlvo() {
        return relatorio.posicaoAlvo;
    }

    /**
     *  Retorna a posição atual medida pelo controlador a cada passo.
     * @return (graus)
     */
    public double obterPosicaoAtual() {
        return relatorio.posicaoMedida;
    }

    /**
     *  Retorna o erro de posição medido pelo controlador a cada passo.
     * @return (graus)
     */
    public double obterErro() {
        return relatorio.erroMedido;
    }

    /**
     *  Retorna o erro de posição medido pelo controlador a cada passo.
     * @return (graus)
     */
    public boolean estaNaPosicaoAlvo() {
        return relatorio.noAlvo;
    }

    /**
     *  Retorna a tensão elétrica enviada pelo controlador a cada passo.
     *  @return (volts)
     */
    public double obterTensaoEnviada() {
        return relatorio.tensaoEletricaRespondida;
    }
}

class tConstantes {
    //Constante de conversão ticks/rotação (rotação = 360º)
    public static double ticksPorRotacao = 1981.935483870968;

    //Limitação do modelo físico (graus, sentido anti-horário)
    public static double limitacaoMaxima = 50;
    public static double limitacaoMinima = -90;

    //Tolerância do controlador (graus)
    public static double tolerancia = Math.toRadians(1);

    //Constantes do FeedForward (kS = volts, kV = volts / graus).
    public static double kV = 1;
    public static double kS = 4;

    //Ganhos do PID (apenas o P e D nesse caso).
    public static double ganhoProporcional = 4;
    public static double ganhoDerivado = 0.1;
}