package frc.robot;

public class Teleop {
    TeleopHelper teleopHelper;
    public Teleop(Driver driver, Shooter shooter, Climber climber) {
        teleopHelper = new TeleopHelper(driver, shooter, climber)
    }
    
    final XboxController driving_xBoxCont = new XboxController(0);

    double contXSpeed = removeDeadzone(1) * XdriveSensitivity;
    double contYSpeed = removeDeadzone(0) * YdriveSensitivity;
    double contTurnSpeed = removeDeadzone(4) * turningSensitivity;



    public double removeDeadzone(int axisInput) {
        if (Math.abs(driving_xBoxCont.getRawAxis(axisInput)) < 0.15) {
            return 0;
        }
        return driving_xBoxCont.getRawAxis(axisInput);
    }

    if (true) {
        contXSpeedField = contXSpeed * Math.cos(botYaw_angleRad) - contYSpeed * Math.sin(botYaw_angleRad);
        contYSpeedField = contXSpeed * Math.sin(botYaw_angleRad) + contYSpeed * Math.cos(botYaw_angleRad);
    }

}
