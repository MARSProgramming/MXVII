package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

public class Rumble {
    private static Rumble mInstance;
    private static XboxController controller1;
    private static XboxController controller2;
    public static Rumble getInstance(){
        if(mInstance == null) mInstance = new Rumble();
        return mInstance;
    }
    private Rumble(){}
    public void setControllers(int c1, int c2){
        controller1 = new XboxController(c1);
        controller2 = new XboxController(c2);
    }
    public void rumbleController1(double rumbleValue){
        controller1.setRumble(RumbleType.kBothRumble, rumbleValue);
    }
    public void rumbleController2(double rumbleValue){
        controller2.setRumble(RumbleType.kBothRumble, rumbleValue);
    }
}
