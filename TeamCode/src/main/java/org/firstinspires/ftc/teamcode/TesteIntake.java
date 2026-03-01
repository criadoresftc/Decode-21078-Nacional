package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.br.Intake;
import org.firstinspires.ftc.teamcode.br.Shooter;
import org.firstinspires.ftc.teamcode.br.Torreta;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class TesteIntake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(120, 128, Math.toRadians(35)));

        follower.startTeleOpDrive();

        CommandScheduler.getInstance().reset();
        CommandScheduler scheduler = CommandScheduler.getInstance();

        Intake intake = new Intake(Intake.Modo.NAO_FAZER_NADA, hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Torreta torreta = new Torreta(0, hardwareMap);

        Telemetry telemetria = PanelsTelemetry.INSTANCE.getTelemetry().getWrapper();
        telemetria.clearAll();

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.x) {
                intake.modo = Intake.Modo.ENVIAR_ARTEFATO;
            } else if(gamepad1.a) {
                intake.modo = Intake.Modo.COLETAR_ARTEFATO;
            } else {
                intake.modo = Intake.Modo.SEGURAR_ARTEFATO;
            }

            if(gamepad1.right_bumper) {
                shooter.velocidade = 1650;
            } else {
                shooter.velocidade = 1400;
            }

            follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            double x = follower.getPose().getX();
            double y = follower.getPose().getY();
            double heading = follower.getPose().getHeading();


            double anguloAlvo = Math.atan2(130 - y, 130 - x);

            double diferencaRad = anguloAlvo - heading;

            torreta.posicao = Math.toDegrees(diferencaRad) + 90;

            intake.adicionarDepuracao(telemetria);
            shooter.adicionarDepuracao(telemetria);

            telemetria.update();
            scheduler.run();
            follower.update();
        }
    }
}
