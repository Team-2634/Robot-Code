package frc.robot.systems;

import java.util.Hashtable;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shuffleboard{
    
    static Map<String, Integer> intMap = new Hashtable<>();
    static Map<String, Boolean> boolMap = new Hashtable<>();


    public void shuffleboard(){
        intMap.put("navx", 0);
        
    }

    public void print(int ex){
        SmartDashboard.putNumber("Number", ex);
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

}
