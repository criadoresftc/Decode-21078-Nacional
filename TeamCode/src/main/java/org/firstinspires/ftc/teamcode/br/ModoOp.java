package org.firstinspires.ftc.teamcode.br;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.br.sis.Intake;
import org.firstinspires.ftc.teamcode.br.sis.Shooter;
import org.firstinspires.ftc.teamcode.br.sis.Torreta;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.WriteAbortedException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Configurable
public abstract class ModoOp extends OpMode {
    public static abstract class Operacao {
        public abstract void iniciar();
        public abstract void loop();
    }
    public class Autonomo extends Operacao {
        private final Command comAtirar =  new ParallelCommandGroup(
                shooter.acelerar(new SupridorVelocidadeDisparoAutomatico()),
                torreta.seguirAlvo(new SupridorDirecaoDisparoAutomatico()),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new WaitUntilCommand(new BooleanSupplier() {
                            @Override
                            public boolean getAsBoolean() {
                                return shooter.estaNaVelocidadeAlvo() && torreta.estaNaPosicaoAlvo();
                            }
                        }),
                        intake.ativar(Intake.Modo.ENVIAR_ARTEFATO)
                )
        ).interruptOn(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !intake.detectarArtefato();
            }
        });

        private final SequentialCommandGroup linhaExecucao = new SequentialCommandGroup();

        public void iniciar() {
            scheduler.schedule(linhaExecucao);
        }
        public void loop() {}

        public Autonomo mover(PathChain path) {
            linhaExecucao.addCommands(new FollowPathCommand(follower, path));
            return this;
        }

        public Autonomo rotacionar(double angulo) {
            linhaExecucao.addCommands(new TurnCommand(follower, angulo, true, AngleUnit.DEGREES));
            return this;
        }

        public Autonomo rotacionarPara(double angulo) {
            linhaExecucao.addCommands(new TurnToCommand(follower, angulo, AngleUnit.DEGREES));
            return this;
        }

        public Autonomo atirar() {
            linhaExecucao.addCommands(comAtirar);
            return this;
        }

        public Autonomo moverEAtirar(PathChain path) {
            linhaExecucao.addCommands(new ParallelCommandGroup(new FollowPathCommand(follower, path), comAtirar));
            return this;
        }

        public Autonomo moverEColetar(PathChain path) {
            linhaExecucao.addCommands(new FollowPathCommand(follower, path).raceWith(intake.ativar(Intake.Modo.COLETAR_ARTEFATO)));
            return this;
        }

        public Autonomo aguardar(long milissegundos) {
            linhaExecucao.addCommands(new WaitCommand(milissegundos));
            return this;
        }
    }
    public class TeleOperado extends Operacao {
        private final Command comAtirar = new ParallelCommandGroup(
                shooter.acelerar(supridorVelocidadeDisparo),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new WaitUntilCommand(new BooleanSupplier() {
                            @Override
                            public boolean getAsBoolean() {
                                return shooter.estaNaVelocidadeAlvo();
                            }
                        }),
                        intake.ativar(Intake.Modo.ENVIAR_ARTEFATO)
                )
        );
        public void iniciar() {
            follower.startTeleOpDrive();

            scheduler.setDefaultCommand(torreta, torreta.seguirAlvo(new SupridorDirecaoDisparoAutomatico()));
        }
        public void loop() {
            follower.setTeleOpDrive(controle1.getLeftY(), -controle1.getLeftX(), controle1.getRightX());
        }

        public TeleOperado mapearAtirar(GamepadButton botao) {
            botao.whileActiveOnce(comAtirar);
            return this;
        }

        public TeleOperado mapearColetar(GamepadButton botao) {
            botao.whileActiveOnce(intake.ativar(Intake.Modo.COLETAR_ARTEFATO));
            return this;
        }

        public TeleOperado mapearEjetar(GamepadButton botao) {
            botao.whileActiveOnce(intake.ativar(Intake.Modo.EJETAR_ARTEFATO));
            return this;
        }

        public TeleOperado mapearDisparoRapido(GamepadButton botao) {
            botao.whileActiveOnce(shooter.acelerar(new SupridorVelocidadeDisparoAutomatico()));
            return this;
        }
    }

    //-- Supridores --

    /**
     *  Calcula a velocidade necessária do Shooter para realizar um disparo ao gol, utilizando uma regressão quadrática.
     *  <p>Supre (ticks/s) necessários para atirar.</p>
     */
    public class SupridorVelocidadeDisparoAutomatico implements DoubleSupplier {
        final double a = 841.6;
        final double b = 6.019;
        final double c = -0.002953;

        @Override
        public double getAsDouble() {
            final double distanciaAlvo = obterDistanciaAlvo();

            return a + b * distanciaAlvo + c * Math.pow(distanciaAlvo, 2);
        }

        /**
         *  Calcula a distância para o gol.
         *
         * @return (polegadas)
         */
        public double obterDistanciaAlvo() {
            return follower.getPose().distanceFrom(alianca.gol);
        }
    }
    /**
     *  Calcula a direção necessária da Torreta para realizar um disparo ao gol.
     *  <p>Supre (graus) necessários para atirar.</p>
     */
    public class SupridorDirecaoDisparoAutomatico implements DoubleSupplier {
        @Override
        public double getAsDouble() {
            return Math.toDegrees(Math.atan2(alianca.gol.getY() - follower.getPose().getY(), alianca.gol.getX() - follower.getPose().getX()) - follower.getHeading()) + 90;
        }
    }

    public enum Alianca {
        AZUL(
                new Pose(1.27, 143),
                new Pose(105, 33.3)),
        VERMELHA(
                new Pose(143, 136),
                new Pose(38.6, 33.3)
        );

        public final Pose gol;
        public final Pose estacionamento;

        Alianca(Pose gol, Pose estacionamento) {
            this.gol = gol;
            this.estacionamento = estacionamento;
        }
    }

    public Operacao operacao;
    public Alianca alianca;

    public Torreta torreta;
    public Shooter shooter;
    public Intake intake;
    public Limelight3A limelight;

    public DoubleSupplier supridorVelocidadeDisparo;

    public GamepadEx controle1;
    public GamepadEx controle2;

    public CommandScheduler scheduler;
    public Follower follower;

    public abstract void construtor();

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        scheduler = CommandScheduler.getInstance();

        follower = Constants.createFollower(hardwareMap);

        telemetry = PanelsTelemetry.INSTANCE.getTelemetry().getWrapper();
        telemetry.clear();

        controle1 = new GamepadEx(gamepad1);
        controle2 = new GamepadEx(gamepad2);

        construtor();
        operacao.iniciar();
    }

    @Override
    public void loop() {
        operacao.loop();

        follower.update();
        scheduler.run();
        telemetry.update();
    }
}
