package frc.robot.systems;

import java.util.Hashtable;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShuffleDash{
    
    static Map<String, Integer> intMap = new Hashtable<>();
    static Map<String, Boolean> boolMap = new Hashtable<>();


    public void shuffledash(){
        boolMap.put("Controller A", false);
    } 

    public static void update(String key, int value){
        intMap.put(key, value);
    }

    public static void update(String key, Boolean value){
        boolMap.put(key, value);
    }

    public static void intLoop(){
        for(Map.Entry<String, Integer> entry : intMap.entrySet()){
            SmartDashboard.putNumber(entry.getKey(), entry.getValue());
        }
    }

    public static void boolLoop(){
        for(Map.Entry<String, Boolean> entry : boolMap.entrySet()){
            SmartDashboard.putBoolean(entry.getKey(), entry.getValue());
        }
    }

}
