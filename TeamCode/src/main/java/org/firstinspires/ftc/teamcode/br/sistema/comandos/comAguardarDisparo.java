package org.firstinspires.ftc.teamcode.br.sistema.comandos;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.br.sistema.Shooter;
import org.firstinspires.ftc.teamcode.br.sistema.Torreta;

/**
 *  Aguarda que a Torreta e o Shooter determinados, alcançem físicamente os seus atríbutos de posição e velocidade.
 */
public class comAguardarDisparo extends CommandBase {
    public static final double TOLERANCIA_VELOCIDADE = 20;
    public static final double TOLERANCIA_POSICAO = Torreta.parametros.tolerancia + 2;

    private final Shooter subShooter;
    private final Torreta subTorreta;

    public comAguardarDisparo(Shooter subShooter, Torreta subTorreta) {
        this.subShooter = subShooter;
        this.subTorreta = subTorreta;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(subShooter.obterVelocidadeRegistrada() - subShooter.velocidade) <= TOLERANCIA_VELOCIDADE && Math.abs(subTorreta.obterPosicaoRegistrada() - MathUtils.normalizeDegrees(subTorreta.posicao, true)) <= TOLERANCIA_POSICAO;
    }
}
