package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;

@Autonomous
public class AutonomoPedroTeste extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(120, 128, Math.toRadians(35)));

        PathChain pathChain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.000, 128.000),
                                new Pose(91.693, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
                .build();


        waitForStart();

        follower.followPath(pathChain);

        while (opModeIsActive()) {
            follower.update();
        }

    }
}
