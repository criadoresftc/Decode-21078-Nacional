package org.firstinspires.ftc.teamcode.br;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.br.sis.Intake;
import org.firstinspires.ftc.teamcode.br.sis.Shooter;
import org.firstinspires.ftc.teamcode.br.sis.Torreta;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Configurable
public class ModoOp extends OpMode {
    /**
     *  Calcula a velocidade necessária do Shooter para realizar um disparo ao gol, utilizando uma regressão quadrática.
     *  <p>Supre (ticks/s) necessários para atirar.</p>
     */
    public class SupridorVelocidadeDisparo implements DoubleSupplier {
        final double a = 0;
        final double b = 0;
        final double c = 0;

        @Override
        public double getAsDouble() {
            final double distanciaAlvo = obterDistanciaAlvo();

            return a + b * distanciaAlvo + c * Math.pow(distanciaAlvo, 2);
        }

        /**
         *  Calcula a distância para o gol.
         *
         * @return (centímetros)
         */
        public double obterDistanciaAlvo() {
            return Math.sqrt(Math.pow(alianca.xGol - follower.getPose().getX(), 2) + Math.pow(alianca.yGol - follower.getPose().getY(), 2));
        }
    }
    /**
     *  Calcula a direção necessária da Torreta para realizar um disparo ao gol.
     *  <p>Supre (graus) necessários para atirar.</p>
     */
    public class SupridorDirecaoDisparo implements DoubleSupplier {
        @Override
        public double getAsDouble() {
            return Math.atan2(alianca.yGol - follower.getPose().getY(), alianca.xGol - follower.getPose().getX()) - follower.getHeading() + 90;
        }
    }

    public enum Alianca {
        AZUL(10, 135, 105, 33.3), VERMELHA(135, 135, 38.6, 33.3);

        public final double xGol;
        public final double yGol;

        public final double xAreaEstacionamento;
        public final double yAreaEstacionamento;

        Alianca(double xGol, double yGol, double xAreaEstacionamento, double yAreaEstacionamento) {
            this.xGol = xGol;
            this.yGol = yGol;
            this.xAreaEstacionamento = xAreaEstacionamento;
            this.yAreaEstacionamento = yAreaEstacionamento;
        }
    }
    public Alianca alianca;

    private CommandScheduler scheduler;
    private Follower follower;

    private Torreta torreta;
    private Shooter shooter;
    private Intake intake;
    private Limelight3A limelight;


    public class Autonomo {
        public Command comAtirar =  new ParallelRaceGroup(
                torreta.seguirAlvo(new SupridorDirecaoDisparo()),
                shooter.acelerar(new SupridorVelocidadeDisparo()).when(new BooleanSupplier() {
                    @Override
                    public boolean getAsBoolean() {
                        return shooter.obterVelocidadeAtual() >= shooter.velocidade;
                    }
                }, intake.ativar(Intake.Modo.ENVIAR_ARTEFATO))
        );

        private final SequentialCommandGroup linhaExecucao = new SequentialCommandGroup();

        public void mover(PathChain path) {
            linhaExecucao.addCommands(new FollowPathCommand(follower, path));
        }

        public void rotacionar(double angulo) {
            linhaExecucao.addCommands(new TurnCommand(follower, angulo, true, AngleUnit.DEGREES));
        }

        public void rotacionarPara(double angulo) {
            linhaExecucao.addCommands(new TurnToCommand(follower, angulo, AngleUnit.DEGREES));
        }

        public void atirar() {
            linhaExecucao.addCommands(
                    new ParallelRaceGroup(
                            torreta.seguirAlvo(new SupridorDirecaoDisparo()),
                            shooter.acelerar(new SupridorVelocidadeDisparo()).when()
                    ).interruptOn(new BooleanSupplier() {
                        @Override
                        public boolean getAsBoolean() {
                            return !intake.detectarArtefato();
                        }
                    })
            );
        }

        public void moverEAtirar(PathChain path) {
            linhaExecucao.addCommands();
        }

        public void moverEColetar(PathChain path) {
            linhaExecucao.addCommands(new ParallelRaceGroup(new FollowPathCommand(follower, path)), intake.ativar(Intake.Modo.COLETAR_ARTEFATO));
        }

        public void aguardar(long milissegundos) {
            linhaExecucao.addCommands(new WaitCommand(milissegundos));
        }
    }


    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        scheduler = CommandScheduler.getInstance();

        follower = Constants.createFollower(hardwareMap);

        telemetry = PanelsTelemetry.INSTANCE.getTelemetry().getWrapper();
        telemetry.clear();
    }

    @Override
    public void loop() {
        follower.update();
        scheduler.run();
        telemetry.update();
    }
}
