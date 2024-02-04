package frc.robot.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class BasicDash {
    private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    private ShuffleboardTab infoTab = Shuffleboard.getTab("Info");
    private Map<String, Supplier<?>> infoOutput;
    private Map<String, SimpleWidget> widgetMap = new HashMap<>();
    public BasicDash(Map<String, String> pilotControls, Map<String, String> copilotControls, Map<String, Supplier<?>> output){
        infoOutput = output;
        ShuffleboardLayout pilotLayout = matchTab.getLayout("Pilot", BuiltInLayouts.kList).withPosition(8, 0).withSize(2, pilotControls.size());
        for(String key : pilotControls.keySet()){
            pilotLayout.add(key, pilotControls.get(key));
        }
        ShuffleboardLayout copilotLayout = matchTab.getLayout("Copilot", BuiltInLayouts.kList).withPosition(10, 0).withSize(2, copilotControls.size());
        for(String key : copilotControls.keySet()){
            copilotLayout.add(key, copilotControls.get(key));
        }

        List<String> sortedKeys = new ArrayList<>(output.keySet());
        Collections.sort(sortedKeys);

        int i = 0;
        for(String key : sortedKeys){
            String name = key;
            widgetMap.put(name, infoTab.add(name, output.get(key).get()).withSize(2, 1).withPosition((i / 5) * 2, i % 5));
            i++;
        }
    }

    public void periodic(){
        for(String key : widgetMap.keySet()){
            widgetMap.get(key).getEntry().setValue(infoOutput.get(key).get());
        }
    }
}
