package frc.robot.constants;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

import frc.robot.util.MoreMath;

public final class RioConstants {
    public static void readConstants(){
        //read swerve zeros from file
        File swerveZeros = new File("/home/lvuser/constants/SwerveZeros.txt");
        if (swerveZeros.exists()) {
            try {
                Scanner sc = new Scanner(swerveZeros);
                Drive.FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(Double.parseDouble(sc.nextLine()));
                Drive.FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(Double.parseDouble(sc.nextLine()));
                Drive.BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(Double.parseDouble(sc.nextLine()));
                Drive.BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(Double.parseDouble(sc.nextLine()));
                sc.close();
            } catch (FileNotFoundException e) {
                System.out.println("Swerve Zeros file not found");
            }
        }
    }
    public static void writeSwerveZeros(double fl, double fr, double bl, double br){
        File swerveZeros = new File("/home/lvuser/constants/SwerveZeros.txt");
        swerveZeros.setExecutable(true);
        swerveZeros.setReadable(true);
        swerveZeros.setWritable(true);
        try {
            swerveZeros.createNewFile();
            FileWriter writer = new FileWriter("/home/lvuser/constants/SwerveZeros.txt");
            writer.write(MoreMath
                    .floorMod(Math.toDegrees(fl
                            - Drive.FRONT_LEFT_MODULE_STEER_OFFSET), 360)
                    + "\n");
            writer.write(MoreMath
                    .floorMod(Math.toDegrees(fr
                            - Drive.FRONT_RIGHT_MODULE_STEER_OFFSET), 360)
                    + "\n");
            writer.write(MoreMath
                    .floorMod(Math.toDegrees(bl
                            - Drive.BACK_LEFT_MODULE_STEER_OFFSET), 360)
                    + "\n");
            writer.write(MoreMath
                    .floorMod(Math.toDegrees(br
                            - Drive.BACK_RIGHT_MODULE_STEER_OFFSET), 360)
                    + "\n");
            writer.close();
        } catch (IOException e) {
            System.out.println("File could not be found when writing to swerve zeros");
        }
    }
    public static class Drive{
        public static double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
        public static double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
        public static double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
        public static double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
    }
}
