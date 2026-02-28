package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.br.Torreta;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class TesteTorretaAutomatica extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(120, 128, Math.toRadians(35)));

        CommandScheduler.getInstance().reset();
        CommandScheduler scheduler = CommandScheduler.getInstance();

        Torreta torreta = new Torreta(0, hardwareMap);

        Telemetry telemetria = PanelsTelemetry.INSTANCE.getTelemetry().getWrapper();
        telemetria.clearAll();

        waitForStart();

        while (opModeIsActive()) {
            torreta.posicao = Math.toDegrees(Math.atan2(120 - follower.getPose().getX(), 128 - follower.getPose().getY()) + follower.getPose().getHeading());
            torreta.adicionarUltimoRelatorio(telemetria);
            telemetria.update();

            follower.update();
            scheduler.run();
        }

    }
}
