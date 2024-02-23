package frc.robot.constants;

public final class StaticConstants {
    public static class Field{
        public static final double fieldLength = 16.58;
        public static final double fieldWidth = 8.2;
    }
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

        public static final double kPSnap = 0.7;
        public static final double kISnap = 0.0;
        public static final double kDSnap = 0.0;

        public static final double kPAlignPiece = 4;
        public static final double kIAlignPiece = 0.0;
        public static final double kDAlignPiece = 0.0;

        public static final double kPTagAlignX = 2.5;
        public static final double kPTagAlignRot = -0.05;
    }

    public static class Auto{
        public static final double holonomicXkP = 3.0;
        public static final double holonomicXkI = 0;
        public static final double holonomicXkD = 0;
        public static final double holonomicYkP = 3.0;
        public static final double holonomicYkI = 0;
        public static final double holonomicYkD = 0;
        public static final double holonomicOkP = 2.0;
        public static final double holonomicOkI = 0.0;
        public static final double holonomicOkD = 0.0;
        public static final double holonomicOMaxVelocity = 5;
        public static final double holonomicOMaxAcceleration = 5;
    }

    public static class IntakePivot{
        public static int ID = 22;

        //Limits are in rotations from zero
        public static double forwardLimit = 0.55;
        public static double reverseLimit = 0.02;
        public static int reverseLimitSwitchID = 0;

        public static double supplyCurrentLimit = 20;
        public static double statorCurrentLimit = 60;
    }
    public static class IntakeWheels{
        public static int ID = 12;
        public static int leftIrID = 2;
        public static int rightIrID = 1;

        public static double supplyCurrentLimit = 20;
        public static double statorCurrentLimit = 20;
    }
    public static class ShooterFlywheel{
        public static int masterID = 13;
        public static int followID = 23;

        public static double supplyCurrentLimit = 20;
        public static double statorCurrentLimit = 40;
    }
    public static class ThePivot{
        public static int ID = 33;

        //Limits are in rotations from zero
        public static double forwardLimit = 0.25;
        public static double reverseLimit = 0.0;

        public static int encoderID = 3;
        public static double supplyCurrentLimit = 20;
        public static double statorCurrentLimit = 40;
    }

    public static class Climber{
        public static int leftID = 18;
        public static int rightID = 28;

        public static int leftLimitSwitchID = 2;
        public static int rightLimitSwitchID = 1;

        public static double leftForwardLimit = 5.6;
        public static double leftReverseLimit = -40;

        public static double rightForwardLimit = 5.6;
        public static double rightReverseLimit = -40;
        
        public static double rightkP = 0.03;
        public static double rightkI = 0.0;
        public static double rightkD = 0.0;


        public static double leftkP = 0.03;
        public static double leftkI = 0.0;
        public static double leftkD = 0.0;
    }
}
