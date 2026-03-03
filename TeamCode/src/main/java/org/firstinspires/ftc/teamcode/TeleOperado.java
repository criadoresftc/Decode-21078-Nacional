package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.br.Robo;
import org.firstinspires.ftc.teamcode.br.sistema.Intake;
import org.firstinspires.ftc.teamcode.br.sistema.comandos.comAguardarDisparo;

@TeleOp
public class TeleOperado extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        if(Robo.INSTANCIA == null) {
            Robo.inicializar(hardwareMap, new Pose(72, 72, Math.toRadians(90)), 90);
        }
        Robo robo = Robo.INSTANCIA;

        robo.definirAlianca(Robo.Alianca.VERMELHA);

        waitForStart();

        GamepadEx controle1 = new GamepadEx(gamepad1);

        robo.follower.startTeleOpDrive();

        while(opModeIsActive()) {
            robo.follower.setTeleOpDrive(controle1.getLeftY(), -controle1.getLeftX(), controle1.getRightX());

            robo.torreta.posicao = Math.toDegrees(Math.atan2(robo.alianca.gol.getY() - robo.follower.getPose().getY(), robo.alianca.gol.getX() - robo.follower.getPose().getX()) - robo.follower.getHeading()) + 90;

            if(controle1.isDown(GamepadKeys.Button.B)) {
                //Coletar artefato
                robo.intake.modo = Intake.Modo.COLETAR_ARTEFATO;
            } else if(controle1.isDown(GamepadKeys.Button.A)) {
                //Atirar
                robo.shooter.velocidade = RegressaoQuadraticaShooter.calc(robo.follower.getPose().distanceFrom(robo.alianca.gol));

                if(Math.abs(robo.shooter.obterVelocidadeRegistrada() - robo.shooter.velocidade) <= comAguardarDisparo.TOLERANCIA_VELOCIDADE) {
                    robo.intake.modo = Intake.Modo.ENVIAR_ARTEFATO;
                }
            } else {
                //Idle
                robo.intake.modo = Intake.Modo.SEGURAR_ARTEFATO;
                robo.shooter.velocidade = 1000;
            }

            robo.scheduler.run();
            robo.follower.update();

            robo.torreta.adicionarDepuracao(robo.telemetria);
            robo.shooter.adicionarDepuracao(robo.telemetria);
            robo.intake.adicionarDepuracao(robo.telemetria);

            robo.telemetria.update();
        }
    }
}
