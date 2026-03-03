package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.br.ModoOp;
import org.firstinspires.ftc.teamcode.br.sis.Intake;
import org.firstinspires.ftc.teamcode.br.sis.Shooter;
import org.firstinspires.ftc.teamcode.br.sis.Torreta;

@TeleOp
public class TesteTeleop extends ModoOp {

    @Override
    public void construtor() {
        shooter = new Shooter(hardwareMap);
        torreta = new Torreta(0, hardwareMap);
        intake = new Intake(Intake.Modo.SEGURAR_ARTEFATO, hardwareMap);

        alianca = Alianca.VERMELHA;

        supridorVelocidadeDisparo = new SupridorVelocidadeDisparoAutomatico();

        follower.setStartingPose(new Pose(120, 128, Math.toRadians(35)));

        operacao = new TeleOperado()
                .mapearColetar(controle2.getGamepadButton(GamepadKeys.Button.B))
                .mapearAtirar(controle2.getGamepadButton(GamepadKeys.Button.A));
    }
}
