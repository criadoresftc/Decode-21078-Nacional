package org.firstinspires.ftc.teamcode.br;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

@Configurable
public class Shooter implements Subsystem {
    public boolean ativo = true;
    public double velocidade = 0;

    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final VoltageSensor sensorEnergia;

    public Shooter(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotorEx.class, "");
        motor2 = hardwareMap.get(DcMotorEx.class, "");

        sensorEnergia = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void periodic() {
        //Aqui defino como o modelo físico deve se comportar diante à mudança dos atributos
        if(ativo) {
            double velocidadeAtual = motor1.getVelocity();

            double voltagemDesejada =
                    (velocidade - velocidadeAtual) * Constantes.ganhoProporcional +
                            velocidade * Constantes.kS + velocidade * Constantes.kV;

            motor1.setPower(voltagemDesejada / sensorEnergia.getVoltage());
            motor2.setPower(voltagemDesejada / sensorEnergia.getVoltage());
        } else {
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }

    /**
     *  Retorna a velocidade real do shooter (ticks/s)
     */
    public double obterVelocidadeReal() {
        return motor1.getVelocity();
    }

    /**
     *  Retorna a aceleração do shooter (ticks/s²)
     */
    private final double[] ultimoRegistroVelocidade = new double[2];
    public double obterAceleracao() {
        double velocidadeAtual = obterVelocidadeReal();
        double tempoAtual = System.currentTimeMillis() * 1e-6;

        double aceleracao = 0;
        if(ultimoRegistroVelocidade.length != 0) {
            aceleracao = (velocidadeAtual - ultimoRegistroVelocidade[0]) / (tempoAtual - ultimoRegistroVelocidade[1]);
        }

        ultimoRegistroVelocidade[0] = velocidadeAtual;
        ultimoRegistroVelocidade[1] = tempoAtual;

        return aceleracao;
    }
}

class Constantes {
    public static double ganhoProporcional = 0;

    public static double kS = 0;
    public static double kV = 0;
}


