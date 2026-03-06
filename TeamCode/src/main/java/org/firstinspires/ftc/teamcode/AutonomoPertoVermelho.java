package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand;

import org.firstinspires.ftc.teamcode.br.Robo;
import org.firstinspires.ftc.teamcode.br.sistema.Intake;
import org.firstinspires.ftc.teamcode.br.sistema.Shooter;
import org.firstinspires.ftc.teamcode.br.sistema.comandos.comAguardarDisparo;
import org.firstinspires.ftc.teamcode.br.sistema.comandos.comEnviarArtefatoEnquantoPossuir;

@Autonomous
public class AutonomoPertoVermelho extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robo.inicializar(hardwareMap, new Pose(120, 128, Math.toRadians(35)), 90);
        Robo robo = Robo.INSTANCIA;

        robo.definirAlianca(Robo.Alianca.VERMELHA);

        PathChain TRSLANAMENTO, POSICIONARPARASEGUNDAFILEIRA, COLETARSEGUNDAFILEIRA, VOLTARPARALANAR, ABRIRGATEECOLETAR, ANDARPARATRSPARACOLETAR, VOLTARPARACHUTAR, COLETARTERCEIRAFILEIRA, VOLTARPARALANCARPARKING;
        TRSLANAMENTO = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.000, 128.000),
                                new Pose(95.000, 96.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();

        POSICIONARPARASEGUNDAFILEIRA = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(95.000, 96.000),
                                new Pose(102.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        COLETARSEGUNDAFILEIRA = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(102.000, 60.000),
                                new Pose(130.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        VOLTARPARALANAR = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130.000, 60.000),
                                new Pose(80.000, 80.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))
                .build();

        ABRIRGATEECOLETAR = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(80.000, 80.000),
                                new Pose(133.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(45))
                .build();

        ANDARPARATRSPARACOLETAR = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.000, 60.000),
                                new Pose(133.000, 55.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(35))
                .build();

        VOLTARPARACHUTAR = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.000, 55.000),
                                new Pose(85.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
                .build();

        COLETARTERCEIRAFILEIRA = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 85.000),
                                new Pose(125.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        VOLTARPARALANCARPARKING = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(125.000, 84.000),
                                new Pose(85.000, 112.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        InstantCommand ativarColeta = new InstantCommand(new Runnable() {
            @Override
            public void run() {
                robo.intake.modo = Intake.Modo.COLETAR_ARTEFATO;
            }
        });

        InstantCommand desativarColeta = new InstantCommand(new Runnable() {
            @Override
            public void run() {
                robo.intake.modo = Intake.Modo.SEGURAR_ARTEFATO;
            }
        });

        InstantCommand ativarShooter = new InstantCommand(new Runnable() {
            @Override
            public void run() {
                robo.shooter.velocidade = RegressaoQuadraticaShooter.calc(robo.follower.getPose().distanceFrom(robo.alianca.gol));
            }
        });

        InstantCommand desativarShooter = new InstantCommand(new Runnable() {
            @Override
            public void run() {
                robo.shooter.velocidade = 1000;
            }
        });

        waitForStart();

        SequentialCommandGroup comando = new SequentialCommandGroup(
                new FollowPathCommand(robo.follower, TRSLANAMENTO),
                ativarShooter,
                new comAguardarDisparo(robo.shooter, robo.torreta),
                new comEnviarArtefatoEnquantoPossuir(robo.intake, Intake.Modo.NAO_FAZER_NADA),
                desativarShooter,
                ativarColeta,
                new FollowPathCommand(robo.follower, POSICIONARPARASEGUNDAFILEIRA),
                new FollowPathCommand(robo.follower, COLETARSEGUNDAFILEIRA),
                new FollowPathCommand(robo.follower, VOLTARPARALANAR),
                desativarColeta,
                ativarShooter,
                new comAguardarDisparo(robo.shooter, robo.torreta),
                new comEnviarArtefatoEnquantoPossuir(robo.intake, Intake.Modo.NAO_FAZER_NADA),
                desativarShooter,
                ativarColeta,
                new FollowPathCommand(robo.follower, ABRIRGATEECOLETAR),
                new FollowPathCommand(robo.follower, ANDARPARATRSPARACOLETAR),
                new WaitCommand(1000),
                ativarShooter,
                new FollowPathCommand(robo.follower, VOLTARPARACHUTAR),
                desativarColeta,
                new comAguardarDisparo(robo.shooter, robo.torreta),
                new comEnviarArtefatoEnquantoPossuir(robo.intake, Intake.Modo.NAO_FAZER_NADA),
                desativarShooter,
                ativarColeta,
                new FollowPathCommand(robo.follower, COLETARTERCEIRAFILEIRA),
                desativarColeta,
                new FollowPathCommand(robo.follower, VOLTARPARALANCARPARKING),
                ativarShooter,
                new comAguardarDisparo(robo.shooter, robo.torreta),
                new comEnviarArtefatoEnquantoPossuir(robo.intake, Intake.Modo.NAO_FAZER_NADA),
                desativarShooter
        );

        robo.scheduler.schedule(comando);
        robo.intake.modo = Intake.Modo.SEGURAR_ARTEFATO;

        while(opModeIsActive()) {
            robo.torreta.posicao = Math.toDegrees(Math.atan2(robo.alianca.gol.getY() - robo.follower.getPose().getY(), robo.alianca.gol.getX() - robo.follower.getPose().getX()) - robo.follower.getHeading()) + 90;

            robo.scheduler.run();
            robo.follower.update();

            robo.torreta.adicionarDepuracao(robo.telemetria);
            robo.shooter.adicionarDepuracao(robo.telemetria);
            robo.intake.adicionarDepuracao(robo.telemetria);

            robo.telemetria.addData("distancia", robo.follower.getPose().distanceFrom(robo.alianca.gol));

            robo.telemetria.update();
        }

        robo.scheduler.cancel(comando);
    }
}
