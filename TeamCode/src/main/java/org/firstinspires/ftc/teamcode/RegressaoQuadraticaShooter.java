package org.firstinspires.ftc.teamcode;

/**
 * Site utilizado:<a href="https://www.omnicalculator.com/statistics/quadratic-regression">...</a>
 */
public class RegressaoQuadraticaShooter {
    public static double a = 768;
    public static double b = 7.942;
    public static double c = -0.0152;

    public static double calc(double valor) {
        return a + b * valor + c * Math.pow(valor, 2);
    }
}
