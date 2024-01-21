package frc.robot.constants;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map.Entry;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public final class DynamicConstants {
    private static HashMap<Field, SimpleWidget> entries;
    public static class Example{
        public static double exampleConstant = 0;
    }

    public static void init(){
        ShuffleboardTab subsystemIOTab;// = Shuffleboard.getTab("SubsystemIO");
        entries = new HashMap<>();

        Class<?>[] subsystems = {Example.class};
        
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
