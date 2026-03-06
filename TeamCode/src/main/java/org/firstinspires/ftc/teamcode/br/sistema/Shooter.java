package org.firstinspires.ftc.teamcode.br.sistema;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

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
        //Constantes do FeedForward (kS -> volts, kV -> volts / ticks/s).
        @Sorter(sort = 1)
        public double kS = 0;
        @Sorter(sort = 2)
        public double kV = 0.0043;

        //Ganhos do PID (apenas o P neste caso).
        @Sorter(sort = 3)
        public double ganhoProporcional = 0.1;
    }
    public static Params parametros = new Params();

    //-- ATRIBUTOS --
    /**
     *  Switch ligar/desligar.
     */
    public boolean ativo = true;
    /**
     *  Velocidade alvo em ticks por segundos.
     */
    public double velocidade = 0;

    //-- ATUADORES E SENSORES --
    private final DcMotorEx motorPrincipal;
    private final DcMotorEx motorSecundario;

    private final Motor.Direction direcaoMotorPrincipal;
    private final Motor.Direction direcaoMotorSecundario;

    private final VoltageSensor sensorEnergia;

    //-- RELATÓRIO --
    //Calcula e armazena informações importantes do último passo do controlador.
    private static class RelatorioControle {
        double velocidadeAlvo, velocidadeMedida, erroMedido, tensaoEletricaRespondida = 0;

        private void atualizar(double velocidadeAlvo, double velocidadeMedida, double tensaoEletricaRespondida) {
            this.velocidadeAlvo = velocidadeAlvo;
            this.velocidadeMedida = velocidadeMedida;
            erroMedido = velocidadeAlvo - velocidadeMedida;
            this.tensaoEletricaRespondida = tensaoEletricaRespondida;
        }
    }
    private final RelatorioControle relatorioControle = new RelatorioControle();

    public Shooter(HardwareMap hardwareMap) {
        motorPrincipal = hardwareMap.get(DcMotorEx.class, "shooter1");
        motorSecundario = hardwareMap.get(DcMotorEx.class, "shooter");

        direcaoMotorPrincipal = Motor.Direction.FORWARD;
        direcaoMotorSecundario = Motor.Direction.FORWARD;

        sensorEnergia = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void periodic() {
        //-- CONTROLADOR --
        //Aqui é definido como o modelo físico deve reagir diante à mudança dos atributos.
        final double velocidadeAtual = motorPrincipal.getVelocity();
        final double velocidadeAlvo = velocidade;

        double tensaoEnviada = 0;
        if(ativo) {
            tensaoEnviada =
                    (velocidadeAlvo - velocidadeAtual) * parametros.ganhoProporcional +
                    Math.signum(velocidadeAlvo) * parametros.kS + velocidadeAlvo * parametros.kV;
        }

        double forca = tensaoEnviada / sensorEnergia.getVoltage();
        motorPrincipal.setPower(forca * direcaoMotorPrincipal.getMultiplier());
        motorSecundario.setPower(forca * direcaoMotorSecundario.getMultiplier());

        relatorioControle.atualizar(velocidadeAlvo, velocidadeAtual, tensaoEnviada);
    }

    /**
     *  Retorna a última velocidade real registrada pelo controlador.
     * @return (ticks/s)
     */
    public double obterVelocidadeRegistrada() {
        return relatorioControle.velocidadeMedida;
    }

    public void adicionarDepuracao(Telemetry telemetria) {
        telemetria.addData(getName().toUpperCase() + " : " + "Vel Alvo (Ticks/s)" , relatorioControle.velocidadeAlvo);
        telemetria.addData(getName().toUpperCase() + " : " + "Vel Atual (Ticks/s)" , relatorioControle.velocidadeMedida);
        telemetria.addData(getName().toUpperCase() + " : " + "Erro (Ticks/s)", relatorioControle.erroMedido);
        telemetria.addData(getName().toUpperCase() + " : " + "Saída (Volts)", relatorioControle.tensaoEletricaRespondida);
    }
}




