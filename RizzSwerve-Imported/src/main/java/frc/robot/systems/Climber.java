package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class Climber {
    TalonFX leftClimb = new TalonFX(Constants.leftClimbID);
    TalonFX rightClimb = new TalonFX(Constants.rightClimbID);

    public void initialize() {
        leftClimb.setNeutralMode(NeutralModeValue.Brake);
        rightClimb.setNeutralMode(NeutralModeValue.Brake);
    }

    public void climb(double input) {
        leftClimb.set(-input);
        rightClimb.set(input);
    }

    public boolean isHardStopped() {
        if (0 < leftClimb.getPosition().getValue() && leftClimb.getPosition().getValue() < Constants.maxClimb) {
            return false;
        } else return true;
    }
}
