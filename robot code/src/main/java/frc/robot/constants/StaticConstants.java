package frc.robot.constants;

public final class StaticConstants {
    public static class Drive{
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.57;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.57;
        public static final double MAX_SPEED_MULTIPLIER = 1.0; 

        public static final int DRIVETRAIN_PIGEON_ID = 31; 
        public static final String kDriveCANivore = "Drivetrain";

            
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 15; 
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5; 
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 25; 

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 14; 
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4; 
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 24;  

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 16; 
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; 
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 26; 

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 17; 
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7; 
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 27; 

        public static final double kP = 5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static class Auto{
        public static final double holonomicXkP = 2.0;
        public static final double holonomicXkI = 0;
        public static final double holonomicXkD = 0;
        public static final double holonomicYkP = 2.4;
        public static final double holonomicYkI = 0;
        public static final double holonomicYkD = 0;
        public static final double holonomicOkP = 2.0;
        public static final double holonomicOkI = 0.0;
        public static final double holonomicOkD = 0.0;
        public static final double holonomicOMaxVelocity = 2;
        public static final double holonomicOMaxAcceleration = 5;
    }

    public static class IntakePivot{
        public static int ID = 22;
        public static double forwardLimit = 0.93;
        public static double reverseLimit = 0.05;
    }
    public static class IntakeWheels{
        public static int ID = 12;
    }
    public static class ShooterFlywheel{
        public static int masterID = 13;
        public static int followID = 23;
    }
    public static class ThePivot{
        public static int ID = 33;
        public static double forwardLimit = 0.4;
        public static double reverseLimit = 0.05;
    }

    public static class Climber{
        public static int leftID = 18;
        public static int rightID = 28;
    }
}
