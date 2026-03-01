package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;

@Autonomous
public class AutonomoPedroTeste extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(120, 128, Math.toRadians(35)));

        PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12;

        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.000, 128.000),
                                new Pose(82.000, 86.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(82.000, 86.000),
                                new Pose(105.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(105.000, 60.000),
                                new Pose(130.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130.000, 60.000),
                                new Pose(83.000, 81.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(83.000, 81.000),
                                new Pose(130.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        Path6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130.000, 60.000),
                                new Pose(127.000, 55.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        Path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(127.000, 55.000),
                                new Pose(84.000, 81.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(84.000, 81.000),
                                new Pose(130.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        Path9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130.000, 60.000),
                                new Pose(127.000, 55.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        Path10 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(127.000, 55.000),
                                new Pose(84.000, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path11 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(84.000, 83.000),
                                new Pose(125.000, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path12 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(125.000, 83.000),
                                new Pose(85.000, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();




        CommandScheduler.getInstance().reset();
        CommandScheduler scheduler = CommandScheduler.getInstance();

        Command comando = new SequentialCommandGroup(
                new FollowPathCommand(follower, Path1),
                new FollowPathCommand(follower, Path2),
                new FollowPathCommand(follower, Path3),
                new FollowPathCommand(follower, Path4),
                new FollowPathCommand(follower, Path5),
                new FollowPathCommand(follower, Path6),
                new FollowPathCommand(follower, Path7),
                new FollowPathCommand(follower, Path8),
                new FollowPathCommand(follower, Path9),
                new FollowPathCommand(follower, Path10),
                new FollowPathCommand(follower, Path11),
                new FollowPathCommand(follower, Path12)
        );

        waitForStart();

        scheduler.schedule(comando);

        while (opModeIsActive()) {
            scheduler.run();
            follower.update();
        }

    }
}
