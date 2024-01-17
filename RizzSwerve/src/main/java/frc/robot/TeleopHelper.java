package frc.robot;

public class TeleopHelper {
    
    Driver driver;
    Shooter shooter;
    Climber climber;

    public TeleopHelper(Driver driver, Shooter shooter, Climber climber) {
        this.driver = driver;
        this.shooter = shooter;
        this.climber = climber;
    }
    
    final XboxController driving_xBoxCont = new XboxController(0);

    

    public void drive(double XSpeed, double YSpeed, double TurnSpeed) {
    swerveDrive(XSpeed, YSpeed, TurnSpeed);
    }

    public double removeDeadzone(int axisInput) {
        if (Math.abs(driving_xBoxCont.getRawAxis(axisInput)) < 0.15) {
            return 0;
        }
        return driving_xBoxCont.getRawAxis(axisInput);
    }

    public void fieldOrientationConvert(double contXSpeed, double contYSpeed) {
        contXSpeedField = contXSpeed * Math.cos(botYaw_angleRad) - contYSpeed * Math.sin(botYaw_angleRad);
        contYSpeedField = contXSpeed * Math.sin(botYaw_angleRad) + contYSpeed * Math.cos(botYaw_angleRad);
    }

}
