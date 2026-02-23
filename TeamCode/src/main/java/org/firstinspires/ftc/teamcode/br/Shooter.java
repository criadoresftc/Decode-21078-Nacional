package org.firstinspires.ftc.teamcode.br;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

//TODO: Testar o subsistema utilizando o robô.

/**
 *  Implementação de um subsistema para o shooter com flywheels do nosso robô.
 *  <br><br>
 *  Baseado em: <a href="https://docs.wpilib.org/pt/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html">...</a>
 *  @author marcosj
 */
@Configurable
public class Shooter extends SubsystemBase {
    private class RelatorioControle {
        //Calcula e armazena informações importantes de um passo do controlador.
        double velocidadeAlvo, velocidadeMedida, aceleracaoMedida, erroMedido, tensaoEletricaRespondida = 0;

        double momento = 0;
        double periodo = 0;

        private void atualizar(double velocidadeAlvo, double velocidadeMedida, double tensaoEletricaRespondida) {
            double ultimoMomentoRegistrado = momento;
            momento = System.nanoTime() * 1e-9;;

            if(ultimoMomentoRegistrado != 0) {
                periodo = (momento - ultimoMomentoRegistrado);
            }

            this.velocidadeAlvo = velocidadeAlvo;

            double ultimaVelocidadeRegistrada = this.velocidadeMedida;
            this.velocidadeMedida = velocidadeMedida;

            if(Math.abs(periodo) > 1e-6) {
                aceleracaoMedida = (velocidadeMedida - ultimaVelocidadeRegistrada) / periodo;
            }

            erroMedido = velocidadeAlvo - velocidadeMedida;

            this.tensaoEletricaRespondida = tensaoEletricaRespondida;
        }
    }

    /**
     *  Switch ligar/desligar.
     */
    public boolean ativo = true;
    /**
     *  Velocidade alvo em ticks por segundos.
     */
    public double velocidade = 0;


    private final DcMotorEx motorPrincipal;
    private final DcMotorEx motorSecundario;

    private final VoltageSensor sensorEnergia;

    private final RelatorioControle relatorio = new RelatorioControle();

    public Shooter(HardwareMap hardwareMap) {
        motorPrincipal = hardwareMap.get(DcMotorEx.class, "");
        motorSecundario = hardwareMap.get(DcMotorEx.class, "");

        sensorEnergia = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void periodic() {
        //-- CONTROLADOR --
        //Aqui é definido como o modelo físico deve reagir diante à mudança dos atributos.

        double velocidadeAtual = motorPrincipal.getVelocity();
        double velocidadeAlvo = velocidade;

        double tensaoEnviada = 0;

        if(ativo) {
            tensaoEnviada =
                    (velocidadeAlvo - velocidadeAtual) * Constantes.ganhoProporcional +
                    velocidadeAlvo * Constantes.kS + velocidadeAlvo * Constantes.kV;
        }

        motorPrincipal.setPower(tensaoEnviada / sensorEnergia.getVoltage());
        motorSecundario.setPower(motorPrincipal.getPower());

        relatorio.atualizar(velocidadeAlvo, velocidadeAtual, tensaoEnviada);
    }

    /**
     *  Retorna a velocidade alvo utilizada pelo controlador a cada passo.
     * @return (ticks/s)
     */
    public double obterVelocidadeAlvo() {
        return relatorio.velocidadeAlvo;
    }
    /**
     *  Retorna a velocidade atual medida pelo controlador a cada passo.
     * @return (ticks/s)
     */
    public double obterVelocidadeAtual() {
        return relatorio.velocidadeMedida;
    }
    /**
     *  Retorna a aceleração medida pelo controlador a cada passo.
     *  @return (ticks/s²)
     */
    public double obterAceleracao() {
       return relatorio.aceleracaoMedida;
    }
    /**
     *  Retorna o erro calculado pelo controlador a cada passo.
     *  @return (ticks/s)
     */
    public double obterErro() {
        return relatorio.erroMedido;
    }
    /**
     *  Retorna a tensão elétrica enviada pelo controlador a cada passo.
     *  @return (volts)
     */
    public double obterTensaoEnviada() {
        return relatorio.tensaoEletricaRespondida;
    }

    /**
     * Acelera o shooter utilizando um supridor de velocidade.
     *
     * <p>Volta para velocidade inicial após o comando ser encerrado.</p>
     *
     * @param velocidadeDesejada A velocidade desejada em (ticks/s)
     * @return (novo Comando)
     */
    public Command acelerar(DoubleSupplier velocidadeDesejada) {
        return new cAcelerar(velocidadeDesejada, this);
    }

    public void adicionarRelatorio(Telemetry telemetria) {
        telemetria.addData(getName().toUpperCase() + " : " + "Vel Alvo (Ticks/s)" , this::obterVelocidadeAlvo);
        telemetria.addData(getName().toUpperCase() + " : " + "Vel Atual (Ticks/s)" , this::obterVelocidadeAtual);
        telemetria.addData(getName().toUpperCase() + " : " + "Aceleração (Ticks/s²)", this::obterAceleracao);
        telemetria.addData(getName().toUpperCase() + " : " + "Erro (Ticks/s)", this::obterErro);
        telemetria.addData(getName().toUpperCase() + " : " + "Resposta (Volts)", this::obterTensaoEnviada);
    }
}

class Constantes {
    //Ganhos do PID (apenas o P nesse caso).
    public static double ganhoProporcional = 0;

    //Constantes do FeedForward (kS = volts, kV = volts / ticks/s).
    public static double kS = 0;
    public static double kV = 0;
}

class cAcelerar extends CommandBase {
    public final Shooter subShooter;
    public final DoubleSupplier velocidadeDesejada;

    private double velocidadeInicial;

    public cAcelerar(DoubleSupplier velocidadeDesejada, Shooter subShooter) {
        this.velocidadeDesejada = velocidadeDesejada;

        this.subShooter = subShooter;
        addRequirements(this.subShooter);
    }

    @Override
    public void initialize() {
        velocidadeInicial = subShooter.velocidade;
    }

    @Override
    public void execute() {
        subShooter.velocidade = velocidadeDesejada.getAsDouble();
    }

    @Override
    public void end(boolean interrupted) {
        subShooter.velocidade = velocidadeInicial;
    }
}



