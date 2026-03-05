package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.br.Robo;
import org.firstinspires.ftc.teamcode.br.sistema.Intake;
import org.firstinspires.ftc.teamcode.br.sistema.comandos.comAguardarDisparo;
import org.firstinspires.ftc.teamcode.br.sistema.comandos.comEnviarArtefatoEnquantoPossuir;

@Autonomous
public class AutonomoLongeAzul extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robo.inicializar(hardwareMap, new Pose(56, 8, Math.toRadians(180)), 90);
        Robo robo = Robo.INSTANCIA;

        robo.definirAlianca(Robo.Alianca.AZUL);

        PathChain Coleta1, Disparo1;
        Coleta1 = robo.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 8.000),
                                new Pose(60.897, 41.298),
                                new Pose(9.737, 35.583)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Disparo1 = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(9.737, 35.583),
                                new Pose(63.145, 19.489)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                ativarShooter,
                new comAguardarDisparo(robo.shooter, robo.torreta),
                new comEnviarArtefatoEnquantoPossuir(robo.intake, Intake.Modo.NAO_FAZER_NADA),
                desativarShooter,
                ativarColeta,
                new FollowPathCommand(robo.follower, Coleta1),
                desativarColeta,
                new FollowPathCommand(robo.follower, Disparo1),
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
    }
}
