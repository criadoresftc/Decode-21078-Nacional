package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.br.Robo;
import org.firstinspires.ftc.teamcode.br.sistema.Intake;
import org.firstinspires.ftc.teamcode.br.sistema.comandos.comAguardarDisparo;
import org.firstinspires.ftc.teamcode.br.sistema.comandos.comEnviarArtefatoEnquantoPossuir;

@Autonomous(group = "auto azul", preselectTeleOp = "TeleOperado")
public class AutonomoLongeAzulJogadorHumano extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robo.inicializar(hardwareMap, new Pose(56, 8, Math.toRadians(180)), 90);
        Robo robo = Robo.INSTANCIA;

        robo.definirAlianca(Robo.Alianca.AZUL);

        PathChain Path1, Path2, Path3, Path4, Path5, Path6;

        Path1 = robo.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 8.000),
                                new Pose(27.000, 11.000),
                                new Pose(10.000, 9.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path2 = robo.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(10.000, 9.000),
                                new Pose(27.000, 11.000),
                                new Pose(56.000, 8.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path3 = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 8.000),
                                new Pose(10.000, 9.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path4 = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.000, 9.000),
                                new Pose(20.000, 11.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path5 = robo.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(20.000, 11.000),
                                new Pose(19.000, 21.000),
                                new Pose(9.000, 25.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(179))
                .build();

        Path6 = robo.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.000, 25.000),
                                new Pose(25.654, 13.892),
                                new Pose(55.000, 8.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                new FollowPathCommand(robo.follower, Path1),
                ativarShooter,
                new FollowPathCommand(robo.follower, Path2),
                desativarColeta,
                new comAguardarDisparo(robo.shooter, robo.torreta),
                new comEnviarArtefatoEnquantoPossuir(robo.intake, Intake.Modo.NAO_FAZER_NADA),
                desativarShooter,
                ativarColeta,
                new FollowPathCommand(robo.follower, Path3),
                new FollowPathCommand(robo.follower, Path4),
                new FollowPathCommand(robo.follower, Path5),
                ativarShooter,
                desativarColeta,
                new FollowPathCommand(robo.follower, Path6),
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
