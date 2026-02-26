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

/*TODO: Testar o subsistema utilizando o robô.
    
*/
/**
 *  Implementação de um subsistema para o shooter com flywheels do nosso robô.
 *  <br><br>
 *  Baseado em: <a href="https://docs.wpilib.org/pt/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html">...</a>
 */
@Configurable
public class Shooter extends SubsystemBase {
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
        motorPrincipal = hardwareMap.get(DcMotorEx.class, "shooter");
        motorSecundario = hardwareMap.get(DcMotorEx.class, "FL");

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
                    (velocidadeAlvo - velocidadeAtual) * sConstantes.ganhoProporcional +
                    Math.signum(velocidadeAlvo) * sConstantes.kS + velocidadeAlvo * sConstantes.kV;
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

    /**
     * Acelera o shooter utilizando um supridor de velocidade.
     *
     * <p>Volta para velocidade inicial após o comando ser encerrado.</p>
     *
     * @param supridorVelocidadeDesejada Supridor da velocidade desejada em (ticks/s)
     * @return (novo Comando)
     */
    public Command acelerar(DoubleSupplier supridorVelocidadeDesejada) {
        return new cAcelerar(supridorVelocidadeDesejada, this);
    }

    public void adicionarUltimoRelatorio(Telemetry telemetria) {
        telemetria.addData(getName().toUpperCase() + " : " + "Vel Alvo (Ticks/s)" , this::obterVelocidadeAlvo);
        telemetria.addData(getName().toUpperCase() + " : " + "Vel Atual (Ticks/s)" , this::obterVelocidadeAtual);
        telemetria.addData(getName().toUpperCase() + " : " + "Erro (Ticks/s)", this::obterErro);
        telemetria.addData(getName().toUpperCase() + " : " + "Saída (Volts)", this::obterSaida);
    }
}

class sConstantes {
    //Constantes do FeedForward (kS = volts, kV = volts / ticks/s).
    @Sorter(sort = 1)
    public static double kS = 0;
    @Sorter(sort = 2)
    public static double kV = 0.0043;

    //Ganhos do PID (apenas o P neste caso).
    @Sorter(sort = 3)
    public static double ganhoProporcional = 0.1;
}

class cAcelerar extends CommandBase {
    public final Shooter subShooter;
    public final DoubleSupplier supridorVelocidadeDesejada;

    private double velocidadeInicial;

    public cAcelerar(DoubleSupplier supridorVelocidadeDesejada, Shooter subShooter) {
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



