package frc.robot.constants;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map.Entry;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public final class DynamicConstants {
    /*
     * Add static classes below for Shuffleboard dynamic contants. Each class represents 
     * a subsystem and will have its own tab. These constants are stored on the RIO through 
     * NetworkTables json and should be saved to the driver station computer after changes.
     * 
     * WARNING: Dynamic constants from the RIO may be read after initialization. Ensure functionality 
     * of any constants that are only read on startup.
     */
    public static class Example{
        public static double exampleConstant = 0;
    }
    public static class Intake{
        //voltage threshold for note detection
        public static double irSensorThreshold = 0.35;

        //Setpoints in rotations from zero
        public static double pivotIntakePosition = 0.6;
        public static double pivotAmpPosition = 0.6;
        public static double pivotUprightPosition = 0.16;
        public static double pivotStowPosition = 0.03;
    }

    public static class ShooterFlywheel{
        public static double testVelocity = 3000;
    }
    public static class ThePivot{
        //Setpoints in rotations from zero
        public static double uprightPosition = 0.325;
        public static double ampPosition = 0.2;
        public static double zeroPosition = 0;

        // setpoints for trap
        public static double trapPosition = 0.2;

        //in volts
        public static double secondSegmentFeedforwardConstant = 0.5;
    }


    private static HashMap<Field, SimpleWidget> entries;

    /*
     * Initializes all the Shuffleboard tabs and widgets by pulling their fields from the provided classes
     */
    public static void init(){
        ShuffleboardTab subsystemIOTab;// = Shuffleboard.getTab("SubsystemIO");
        entries = new HashMap<>();

        //add all .class values of the static classes above
        Class<?>[] subsystems = {Intake.class, ThePivot.class, ShooterFlywheel.class};
        
        for(Class<?> subsystem : subsystems){
            Field[] fields = subsystem.getDeclaredFields();
            subsystemIOTab = Shuffleboard.getTab(subsystem.getSimpleName());
            for (Field field : fields) {
                if (java.lang.reflect.Modifier.isStatic(field.getModifiers()) && field.getType() == double.class) {
                    field.setAccessible(true);
                    double value = 0;
                    try{
                        value = field.getDouble(null);
                    }
                    catch(IllegalAccessException a){
                        System.out.println("Access Exception when reading subsystem IO");
                    }
                    entries.put(field, subsystemIOTab.addPersistent(subsystem.getSimpleName() + ": " + field.getName(), "double", value).withSize(2, 1));
                }
            }
        }
    }

    /*
     * Must be periodically called. Updates the constants values from the Shuffleboard
     */
    public static void periodic(){
        for(Entry<Field, SimpleWidget> entry : entries.entrySet()){
            try{
                entry.getKey().setDouble(null, entry.getValue().getEntry().getDouble(0));
            }
            catch(IllegalAccessException a){
                System.out.println("Access Exception when writing subsystem IO");
            }
        }
    }
}
