package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.br.Torreta;

@TeleOp
public class TesteTorretaAutomatica extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        CommandScheduler scheduler = CommandScheduler.getInstance();

        Torreta torreta = new Torreta(0, hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            torreta.moverPara(10);
            telemetry.addData("a", torreta.obterPosicaoAtual());
            telemetry.addData("b", torreta.encoder.getCurrentPosition());
            telemetry.update();

            scheduler.run();
        }

    }
}
