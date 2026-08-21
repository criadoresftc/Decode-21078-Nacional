package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
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

@Autonomous(group = "auto azul", preselectTeleOp = "TeleOperado")
public class AutonomoSplinePertoAzul extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robo.inicializar(hardwareMap, new Pose(24, 128, Math.toRadians(145)), 90);
        Robo robo = Robo.INSTANCIA;

        robo.definirAlianca(Robo.Alianca.AZUL);

        PathChain ANDARPARATRSELANAR, COLETARPRIMEIRAFILA, VOLTARPARALANAR, ABRIRGATE, COLETARGATE, VOLTARPARALANAR2, COLETARSEGUNDAFILA, VOLTARPARALANAR3, LEAVE;

        ANDARPARATRSELANAR = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(24.000, 128.000),
                                new Pose(53.000, 93.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        COLETARPRIMEIRAFILA = robo.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.000, 93.000),
                                new Pose(53.000, 56.000),
                                new Pose(19.000, 59.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        VOLTARPARALANAR = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(19.000, 59.000),
                                new Pose(60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        ABRIRGATE = robo.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(39.000, 60.000),
                                new Pose(16.000, 70.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                .build();

        COLETARGATE = robo.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(16.000, 70.000),
                                new Pose(20.873, 54.761),
                                new Pose(13.000, 52.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(135))
                .build();

        VOLTARPARALANAR2 = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(13.000, 52.000),
                                new Pose(60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        COLETARSEGUNDAFILA = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.000, 84.000),
                                new Pose(19.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        VOLTARPARALANAR3 = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(19.000, 84.000),
                                new Pose(60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        LEAVE = robo.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.000, 84.000),
                                new Pose(39.000, 74.000)
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
                new FollowPathCommand(robo.follower, ANDARPARATRSELANAR),
                ativarShooter,
                new comAguardarDisparo(robo.shooter, robo.torreta),
                new comEnviarArtefatoEnquantoPossuir(robo.intake, Intake.Modo.NAO_FAZER_NADA),
                desativarShooter,
                ativarColeta,
                new FollowPathCommand(robo.follower, COLETARPRIMEIRAFILA),
                ativarShooter,
                new FollowPathCommand(robo.follower, VOLTARPARALANAR),
                desativarColeta,
                new comAguardarDisparo(robo.shooter, robo.torreta),
                new comEnviarArtefatoEnquantoPossuir(robo.intake, Intake.Modo.NAO_FAZER_NADA),
                ativarColeta,
                new FollowPathCommand(robo.follower, ABRIRGATE),
                new FollowPathCommand(robo.follower, COLETARGATE),
                ativarShooter,
                new FollowPathCommand(robo.follower, VOLTARPARALANAR2),
                desativarColeta,
                new comAguardarDisparo(robo.shooter, robo.torreta),
                new comEnviarArtefatoEnquantoPossuir(robo.intake, Intake.Modo.NAO_FAZER_NADA),
                desativarShooter,
                ativarColeta,
                new FollowPathCommand(robo.follower, COLETARSEGUNDAFILA),
                ativarShooter,
                new FollowPathCommand(robo.follower, VOLTARPARALANAR3),
                desativarColeta,
                new comAguardarDisparo(robo.shooter, robo.torreta),
                new comEnviarArtefatoEnquantoPossuir(robo.intake, Intake.Modo.NAO_FAZER_NADA),
                desativarShooter,
                new FollowPathCommand(robo.follower, LEAVE)
        );

        robo.scheduler.schedule(comando);
        robo.intake.modo = Intake.Modo.SEGURAR_ARTEFATO;

        robo.limelight.start();

        while(opModeIsActive()) {
            LLResult result = robo.limelight.getLatestResult();
            if(result.isValid()) {
                robo.torreta.posicao = robo.torreta.obterPosicaoRegistrada() - result.getTx();
            } else {
                robo.torreta.posicao = Math.toDegrees(Math.atan2(robo.alianca.gol.getY() - robo.follower.getPose().getY(), robo.alianca.gol.getX() - robo.follower.getPose().getX()) - robo.follower.getHeading()) + 90;
            }

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
