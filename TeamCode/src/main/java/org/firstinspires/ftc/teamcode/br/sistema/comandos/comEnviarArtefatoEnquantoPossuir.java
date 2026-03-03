package org.firstinspires.ftc.teamcode.br.sistema.comandos;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.br.sistema.Intake;

/**
 *  Submete o Intake determinado com o modo Enviar, enquanto um artefato for detectado pelo mesmo.
 *
 *  <p>Após o fim, submete o Intake no Modo de Encerramento determinado.</p>
 */
public class comEnviarArtefatoEnquantoPossuir extends CommandBase {
    public final Intake.Modo modoEncerramento;

    private final Intake subIntake;
    public comEnviarArtefatoEnquantoPossuir(Intake subIntake, Intake.Modo modoEncerramento) {
        this.modoEncerramento = modoEncerramento;

        this.subIntake = subIntake;
    }

    @Override
    public void initialize() {
        subIntake.modo = Intake.Modo.ENVIAR_ARTEFATO;
    }

    @Override
    public void end(boolean interrupted) {
        subIntake.modo = modoEncerramento;
    }

    @Override
    public boolean isFinished() {
        return !subIntake.detectarArtefato();
    }
}
