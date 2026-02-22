package org.firstinspires.ftc.teamcode.NEWRO.Processors;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;

@Config
public class PIDClassForTele {
    private static PIDFController controller = new PIDFController(0, 0, 0, 0);

    public static double p = 0.07, i = 0.12, d= 0;
    public static double f = 0.00001 ;

    private static final double ticks_in_degree = 700.0 / 180.0;

    static double power;

    public static double returnRevPID(double target, double revpose) {
        controller.setPIDF(p, i, d, f);

        double pid = controller.calculate(revpose, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        controller.setTolerance(0.5);
        controller.atSetPoint();

        power = pid + ff;

        return power;
    }
}
