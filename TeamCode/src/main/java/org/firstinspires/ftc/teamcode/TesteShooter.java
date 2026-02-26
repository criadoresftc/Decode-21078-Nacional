package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ftc.FTCCoordinates;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.br.Shooter;

import java.util.function.DoubleSupplier;

@TeleOp
public class TesteShooter extends LinearOpMode {
    public final double velocidadeMaxima = 1800;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        CommandScheduler scheduler = CommandScheduler.getInstance();

        Shooter shooter = new Shooter(hardwareMap);

        GamepadEx op1 = new GamepadEx(gamepad1);
        DoubleSupplier velocidade = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return op1.getLeftX() * velocidadeMaxima;
            }
        };

        scheduler.schedule(shooter.acelerar(velocidade));

        Telemetry telemetria = PanelsTelemetry.INSTANCE.getTelemetry().getWrapper();
        telemetria.clearAll();

        waitForStart();

        while(opModeIsActive()) {
            shooter.velocidade = 500;
            scheduler.run();

            shooter.adicionarUltimoRelatorio(telemetria);
            telemetria.update();
        }

        scheduler.cancelAll();
    }
}
