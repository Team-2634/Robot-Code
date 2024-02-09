package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDash{
    
    public void print(int ex){
        SmartDashboard.putNumber("Number", ex);
    }

}
