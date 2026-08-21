package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.br.Robo;

@TeleOp
@Configurable
public class TesteRegressao extends LinearOpMode {
    public static double velocidade = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Robo.inicializar(hardwareMap, new Pose(56, 8, Math.toRadians(180)), 90);
        Robo robo = Robo.INSTANCIA;

        robo.definirAlianca(Robo.Alianca.AZUL);

        waitForStart();

        robo.limelight.start();

        while (opModeIsActive()) {
            LLResult result = robo.limelight.getLatestResult();
            if(result.isValid()) {
                robo.torreta.posicao = robo.torreta.obterPosicaoRegistrada() - result.getTx();
            } else {
                robo.torreta.posicao = Math.toDegrees(Math.atan2(robo.alianca.gol.getY() - robo.follower.getPose().getY(), robo.alianca.gol.getX() - robo.follower.getPose().getX()) - robo.follower.getHeading()) + 90;
            }

            robo.shooter.velocidade = velocidade;

            robo.scheduler.run();
            robo.follower.update();

            robo.telemetria.addData("distancia", robo.follower.getPose().distanceFrom(robo.alianca.gol));

            robo.torreta.adicionarDepuracao(robo.telemetria);
            robo.shooter.adicionarDepuracao(robo.telemetria);
            robo.intake.adicionarDepuracao(robo.telemetria);


            robo.telemetria.update();
        }
    }
}
