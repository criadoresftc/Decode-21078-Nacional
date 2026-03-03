package org.firstinspires.ftc.teamcode;

/**
 * Site utilizado:<a href="https://www.omnicalculator.com/statistics/quadratic-regression">...</a>
 */
public class RegressaoQuadraticaShooter {
    public static double a = 841.2;
    public static double b = 6.019;
    public static double c = -0.002953;

    public static double calc(double valor) {
        return a + b * valor + c * Math.pow(valor, 2);
    }
}
