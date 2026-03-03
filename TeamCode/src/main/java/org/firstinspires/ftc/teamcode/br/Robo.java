package org.firstinspires.ftc.teamcode.br;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.br.sistema.Intake;
import org.firstinspires.ftc.teamcode.br.sistema.Shooter;
import org.firstinspires.ftc.teamcode.br.sistema.Torreta;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


/**
 *  Configuração do nosso robô na arena.
 */
public class Robo {
    public static Robo INSTANCIA = null;

    public CommandScheduler scheduler;
    public Follower follower;
    public Telemetry telemetria;

    public Torreta torreta;
    public Shooter shooter;
    public Intake intake;

    /**
     * Cria uma nova instância para determinada posição inicial.
     * @param hardwareMap HardwareMap
     * @param poseInicial Pose inicial do robô na arena.
     * @param posicaoInicialTorreta Posição inicial da torreta na arena.
     */
    public static void inicializar(HardwareMap hardwareMap, Pose poseInicial, double posicaoInicialTorreta) {
        INSTANCIA = new Robo(hardwareMap);

        INSTANCIA.follower.setStartingPose(poseInicial);
        INSTANCIA.torreta.posicaoInicial = posicaoInicialTorreta;
    }

    private Robo(HardwareMap hardwareMap) {
        CommandScheduler.getInstance().reset();
        scheduler = CommandScheduler.getInstance();

        follower = Constants.createFollower(hardwareMap);

        telemetria = PanelsTelemetry.INSTANCE.getTelemetry().getWrapper();
        telemetria.clearAll();

        torreta = new Torreta(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    //-- MODIFICADORES --
    public enum Alianca {
        AZUL(new Pose(4, 140)), VERMELHA(new Pose(140, 140));

        public final Pose gol;

        Alianca(Pose gol) {
            this.gol = gol;
        }
    }
    public Alianca alianca = Alianca.VERMELHA;

    /**
     * Define a aliança pertencente.
     */
    public void definirAlianca(Alianca alianca) {
        this.alianca = alianca;
    }
}
