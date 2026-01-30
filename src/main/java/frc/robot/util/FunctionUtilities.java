package frc.robot.util;

public class FunctionUtilities {
     public static double applyClamp(double input, double minimum, double maximum) {
        if (input < minimum) return minimum;
        else if (input > maximum) return maximum;
        else return input;
    }
}
