package frc.robot.util;

public class MoreMath {
    public static double floorMod(double x, double y){
        return (x - Math.floor(x/y) * y);
    }
}
