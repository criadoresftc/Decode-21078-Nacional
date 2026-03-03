package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.br.ModoOp;
import org.firstinspires.ftc.teamcode.br.sis.Intake;
import org.firstinspires.ftc.teamcode.br.sis.Shooter;
import org.firstinspires.ftc.teamcode.br.sis.Torreta;

@Autonomous
public class TesteAutonomoAzul extends ModoOp {
    @Override
    public void construtor() {
        torreta = new Torreta(0, hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(Intake.Modo.SEGURAR_ARTEFATO, hardwareMap);

        alianca = Alianca.AZUL;

        follower.setStartingPose(new Pose(56, 8, Math.toRadians(180)));

        supridorVelocidadeDisparo = new SupridorVelocidadeDisparoAutomatico();

        PathChain Path1, Path2;

        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 8.000),
                                new Pose(63.773, 42.172),
                                new Pose(5.360, 36.417)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        operacao = new Autonomo()
                .atirar();

    }
}
