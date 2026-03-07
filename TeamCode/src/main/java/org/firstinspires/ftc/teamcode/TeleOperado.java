package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;

import org.firstinspires.ftc.teamcode.br.Robo;
import org.firstinspires.ftc.teamcode.br.sistema.Intake;
import org.firstinspires.ftc.teamcode.br.sistema.comandos.comAguardarDisparo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class TeleOperado extends LinearOpMode {
    public final double ZONA_MORTA = 0.6;
    public final double SENSIBILIDADE_TRIGGER = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {
        Robo robo = Robo.INSTANCIA;

        Servo angulador = hardwareMap.get(Servo.class, "a");

        waitForStart();

        GamepadEx controle1 = new GamepadEx(gamepad1);
        GamepadEx controle2 = new GamepadEx(gamepad2);

        robo.follower.startTeleOpDrive();
        robo.follower.setMaxPower(0.8);

        angulador.setPosition(0);

        double offsetAlinhamento = 0;
        double offsetVelocidadeShooter = 0;

        while(opModeIsActive()) {
            robo.follower.setTeleOpDrive(controle1.getLeftY(), -controle1.getLeftX(), controle1.getRightX());

            angulador.setPosition(0);

            //Torreta
            /*
            double dirStick = Math.toDegrees(Math.atan2(-controle2.getRightY(), controle2.getRightX()));
            if(Math.sqrt(Math.pow(controle2.getRightX(), 2) + Math.pow(controle2.getRightY(), 2)) > 1.414213 * ZONA_MORTA) {
                robo.torreta.posicao = dirStick;
            } else {
                robo.torreta.posicao = Math.toDegrees(Math.atan2(robo.alianca.gol.getY() - robo.follower.getPose().getY(), robo.alianca.gol.getX() - robo.follower.getPose().getX()) - robo.follower.getHeading()) + 90;
            }*/

            //Torreta
            robo.torreta.posicao = Math.toDegrees(Math.atan2(robo.alianca.gol.getY() - robo.follower.getPose().getY(), robo.alianca.gol.getX() - robo.follower.getPose().getX()) - robo.follower.getHeading() + offsetAlinhamento) + 90;

            if(controle2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > SENSIBILIDADE_TRIGGER) {
                //Atirar
                robo.shooter.velocidade = RegressaoQuadraticaShooter.calc(robo.follower.getPose().distanceFrom(robo.alianca.gol)) + offsetVelocidadeShooter;

                if (Math.abs(robo.shooter.obterVelocidadeRegistrada() - robo.shooter.velocidade) <= comAguardarDisparo.TOLERANCIA_VELOCIDADE) {
                    robo.intake.modo = Intake.Modo.ENVIAR_ARTEFATO;
                }
            }
            else if(controle1.isDown(GamepadKeys.Button.A) || controle2.isDown(GamepadKeys.Button.A)) {
                //Coletar artefato
                robo.intake.modo = Intake.Modo.COLETAR_ARTEFATO;
            } else {
                //Idle
                robo.intake.modo = Intake.Modo.SEGURAR_ARTEFATO;
                robo.shooter.velocidade = 1000;
            }

            //Ajuste de offset
            if(controle2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                offsetAlinhamento += 0.05;
            }
            if(controle2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                offsetAlinhamento -= 0.05;
            }
            if(controle2.wasJustPressed(GamepadKeys.Button.X)) {
                offsetAlinhamento = 0;
            }

            if(controle2.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
                offsetVelocidadeShooter += 20;
            }
            if(controle2.isDown(GamepadKeys.Button.DPAD_LEFT)) {
                offsetVelocidadeShooter -= 20;
            }
            if(controle2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                offsetVelocidadeShooter = 0;
            }

            //Relocalização manual
            if(controle2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                robo.follower = Constants.createFollower(hardwareMap);

                if(robo.alianca == Robo.Alianca.AZUL) {
                    robo.follower.setStartingPose(new Pose(24, 128, Math.toRadians(145)));
                } else {
                    robo.follower.setStartingPose(new Pose(120, 128, Math.toRadians(35)));
                }
                robo.follower.startTeleOpDrive();
                robo.follower.setMaxPower(0.8);
            }

            robo.scheduler.run();
            robo.follower.update();

            robo.torreta.adicionarDepuracao(robo.telemetria);
            robo.shooter.adicionarDepuracao(robo.telemetria);
            robo.intake.adicionarDepuracao(robo.telemetria);

            robo.telemetria.addData("alianca", robo.alianca);
            robo.telemetria.addData("posicao", robo.torreta.posicao);
            robo.telemetria.addData("posex", robo.follower.getPose().getX());
            robo.telemetria.addData("heading", robo.follower.getHeading());

            robo.telemetria.update();
        }
    }
}
