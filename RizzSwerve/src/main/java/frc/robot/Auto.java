package frc.robot;
/*
 * 
 * 
 */
public class Auto {
    AutoHelper autoHelper;
    public Auto(Driver driver, Shooter shooter, Climber climber) {
        this.autoHelper = new AutoHelper(driver, shooter, climber)
    }
    
}
