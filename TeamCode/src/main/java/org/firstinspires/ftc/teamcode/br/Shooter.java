package org.firstinspires.ftc.teamcode.br;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;


/**
 *  Implementação de um subsistema para o Shooter com Flywheels do nosso robô.
 *  <br><br>
 *  Baseado em: <a href="https://docs.wpilib.org/pt/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html">...</a>
 */
@Configurable
public class Shooter extends SubsystemBase {
    //-- PARÂMETROS --

    public static class Params {
        //Constantes do FeedForward (kS = volts, kV = volts / ticks/s).
        @Sorter(sort = 1)
        public double kS = 0;
        @Sorter(sort = 2)
        public double kV = 0.0043;

        //Ganhos do PID (apenas o P neste caso).
        @Sorter(sort = 3)
        public double ganhoProporcional = 0.1;
    }
    public static Params parametros = new Params();

    private static class RelatorioControle {
        //Calcula e armazena informações importantes de um passo do controlador.
        double velocidadeAlvo, velocidadeMedida, erroMedido, tensaoEletricaRespondida = 0;

        double momento = 0;

        private void atualizar(double velocidadeAlvo, double velocidadeMedida, double tensaoEletricaRespondida, double momento) {
            this.velocidadeAlvo = velocidadeAlvo;
            this.velocidadeMedida = velocidadeMedida;
            erroMedido = velocidadeAlvo - velocidadeMedida;
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
     *  Velocidade alvo em ticks por segundos.
     */
    public double velocidade = 0;

    private final DcMotorEx motorPrincipal;
    private final DcMotorEx motorSecundario;

    private final VoltageSensor sensorEnergia;

    private final RelatorioControle ultimoRelatorio = new RelatorioControle();

    public Shooter(HardwareMap hardwareMap) {
        motorPrincipal = hardwareMap.get(DcMotorEx.class, "shooter1");
        motorSecundario = hardwareMap.get(DcMotorEx.class, "shooter");

        sensorEnergia = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void periodic() {
        //-- CONTROLADOR --
        //Aqui é definido como o modelo físico deve reagir diante à mudança dos atributos.
        
        final double velocidadeAtual = motorPrincipal.getVelocity();
        final double velocidadeAlvo = velocidade;
        
        final double momento = System.currentTimeMillis() / 1E6;

        double tensaoEnviada = 0;
        if(ativo) {
            tensaoEnviada =
                    (velocidadeAlvo - velocidadeAtual) * parametros.ganhoProporcional +
                    Math.signum(velocidadeAlvo) * parametros.kS + velocidadeAlvo * parametros.kV;
        }

        motorPrincipal.setPower(tensaoEnviada / sensorEnergia.getVoltage());
        motorSecundario.setPower(motorPrincipal.getPower());

        ultimoRelatorio.atualizar(velocidadeAlvo, velocidadeAtual, tensaoEnviada, momento);
    }

    /**
     *  Retorna a velocidade alvo utilizada pelo controlador no último passo.
     * @return (ticks/s)
     */
    public double obterVelocidadeAlvo() {
        return ultimoRelatorio.velocidadeAlvo;
    }
    /**
     *  Retorna a velocidade atual medida pelo controlador no último passo
     * @return (ticks/s)
     */
    public double obterVelocidadeAtual() {
        return ultimoRelatorio.velocidadeMedida;
    }
    /**
     *  Retorna o erro calculado pelo controlador no último passo.
     *  @return (ticks/s)
     */
    public double obterErro() {
        return ultimoRelatorio.erroMedido;
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
     * Acelera o shooter utilizando um supridor de velocidade.
     *
     * <p>Volta para velocidade inicial após o comando ser encerrado.</p>
     *
     * @param supridorVelocidadeDesejada Supridor da velocidade desejada em (ticks/s)
     * @return (novo Comando)
     */
    public Command acelerar(DoubleSupplier supridorVelocidadeDesejada) {
        return new ComAcelerar(supridorVelocidadeDesejada, this);
    }
    /**
     * Segura o shooter numa determinada velocidade.
     *
     * <p>Volta para velocidade inicial após o comando ser encerrado.</p>
     *
     * @param velocidadeDesejada A velocidade desejada em (ticks/s)
     * @return (novo Comando)
     */
    public Command segurarVelocidade(double velocidadeDesejada) {
        return new ComSegurarVelocidade(velocidadeDesejada, this);
    }

    public void adicionarDepuracao(Telemetry telemetria) {
        telemetria.addData(getName().toUpperCase() + " : " + "Vel Alvo (Ticks/s)" , this::obterVelocidadeAlvo);
        telemetria.addData(getName().toUpperCase() + " : " + "Vel Atual (Ticks/s)" , this::obterVelocidadeAtual);
        telemetria.addData(getName().toUpperCase() + " : " + "Erro (Ticks/s)", this::obterErro);
        telemetria.addData(getName().toUpperCase() + " : " + "Saída (Volts)", this::obterSaida);
    }
}

class ComAcelerar extends CommandBase {
    public final Shooter subShooter;
    public final DoubleSupplier supridorVelocidadeDesejada;

    private double velocidadeInicial;

    public ComAcelerar(DoubleSupplier supridorVelocidadeDesejada, Shooter subShooter) {
        this.supridorVelocidadeDesejada = supridorVelocidadeDesejada;

        this.subShooter = subShooter;
        addRequirements(this.subShooter);
    }

    @Override
    public void initialize() {
        velocidadeInicial = subShooter.velocidade;
    }

    @Override
    public void execute() {
        subShooter.velocidade = supridorVelocidadeDesejada.getAsDouble();
    }

    @Override
    public void end(boolean interrupted) {
        subShooter.velocidade = velocidadeInicial;
    }
}

class ComSegurarVelocidade extends CommandBase {
    public final Shooter subShooter;
    public final double velocidadeDesejada;

    private double velocidadeInicial;

    public ComSegurarVelocidade(double velocidadeDesejada, Shooter subShooter) {
        this.velocidadeDesejada = velocidadeDesejada;

        this.subShooter = subShooter;
        addRequirements(this.subShooter);
    }

    @Override
    public void initialize() {
        velocidadeInicial = subShooter.velocidade;

        subShooter.velocidade = velocidadeDesejada;
    }

    public void end(boolean interrupted) {
        subShooter.velocidade = velocidadeInicial;
    }
}



