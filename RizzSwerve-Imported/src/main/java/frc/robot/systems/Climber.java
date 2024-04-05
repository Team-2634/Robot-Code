package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Climber {
    public TalonFX leftClimb = new TalonFX(Constants.leftClimbID);
    public TalonFX rightClimb = new TalonFX(Constants.rightClimbID);

    public void initialize() {
        leftClimb.setNeutralMode(NeutralModeValue.Brake);
        rightClimb.setNeutralMode(NeutralModeValue.Brake);

        leftClimb.setInverted(true);
        rightClimb.setInverted(false);
    }

    public void climb(double input) {
        leftClimb.set(input);
        rightClimb.set(input);
    }

    public double getClimbPosition() {
        return rightClimb.getPosition().getValue();
    }

    public boolean isHardStoppedLow() {
        if (Constants.minClimb < leftClimb.getPosition().getValue()) {
            return false;
        } else return true;
    }

    public boolean isHardStoppedHigh() {
        if (leftClimb.getPosition().getValue() < Constants.maxClimb) {
            return false;
        } else return true;
    }
}
