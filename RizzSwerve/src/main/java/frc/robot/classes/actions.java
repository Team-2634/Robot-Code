package frc.robot.classes;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public class actions implements Runnable {

    // TODO this will be the list of basic auto actions, ex. DriveDist, TurnDeg,
    // Balance...
    // In an additional ActionHandler(?) class we will have a series of
    // ActionGroups(?) which are the assembled auto code ex. DrivePlaceTurnDrive
    // THe one to be used will be selected inside main

    //Todo: Dislike
    String functionName;
    double parameter;

    SwerveDriveTrain driveTrain;
    AHRS navx;

    // TODO make arm piece w/ wrist, elbow, shoulder, fingers, extensions?
    Motor shouldeMotor; //TODO make Drivetrain/motorgroup
    Motor armExtension;

    public actions(SwerveDriveTrain driveTrain, AHRS navx, Motor shouldeMotor, Motor armExtension, String functionName, double parameter) {
        this.driveTrain = driveTrain;
        this.navx = navx;
        
        this.functionName = functionName;
        this.parameter = parameter;

        this.shouldeMotor = shouldeMotor;
        this.armExtension = armExtension;
    }
    
    //TODO Helper functions, move elsewhere
    double getEncoderX(){
        return driveTrain.getDriveEncoderPosition(MotorLocation.FrontLeft_Drive);
    }
    double getBotYaw(){
        return this.navx.getPitch();
    }

    public void run() {
        switch (functionName) {
            case ("driveDistance"):
                driveDistance(parameter, getEncoderX());
                break;
            case ("driveRotate"):
                driveRotate(parameter, getBotYaw());
                break;
            case ("placeThingy"):
                // placeThingy(15);
                break;
            case ("autoMidBalance"):
                driveRotate(Math.PI - 0.01, getBotYaw());
                driveTrain.setDriveEncoderPosition(0);;
        
                break;
            case ("driveBackwardsThenRotate"):
                driveDistance(parameter, getEncoderX());
                driveRotate(180, getBotYaw());
                break;
            default:
                System.out.println("ERROR, command not found: " + functionName);
        }
    }

    public boolean driveDistance(double targetX_Meters, double encoderDisplacement_Meters) {
        ActionHelpers.action("Drive to " + targetX_Meters);
        targetX_Meters = -targetX_Meters; // TODO why Inverse???

        double currentDistanceX = encoderDisplacement_Meters;

        // TODO config
        double tolerance = 0.05;
        double xSpeed = 0.45;
        double xSpeed_Rev = -0.30;

        if (Math.abs(targetX_Meters - currentDistanceX) > tolerance) {
            if (currentDistanceX < targetX_Meters) {
                driveTrain.GO(xSpeed, 0, 0);
                System.out.println("behind target");
                return false;
            }
            if (currentDistanceX > targetX_Meters) {
                driveTrain.GO(-xSpeed_Rev, 0, 0);
                System.out.println("in front of target");
                return false;
            }
            return false;
        } else {
            driveTrain.GO(0, 0, 0);
            ActionHelpers.action("Destination Reached");
            return true;
        }
    }

    public boolean driveRotate(double targetYaw_Rad, double botYaw_Rad) {
        ActionHelpers.action("Rotating to " + targetYaw_Rad);

        double currentRoationYaw_Rad = botYaw_Rad;

        // TODO config
        double tolerance = 0.2;
        double RotSpeed = 25; // rads per sec

        if (Math.abs(targetYaw_Rad - currentRoationYaw_Rad) > tolerance) {
            if (currentRoationYaw_Rad < targetYaw_Rad) {
                driveTrain.GO(0, 0, RotSpeed);
                return false;
            } else if (currentRoationYaw_Rad > targetYaw_Rad) {
                driveTrain.GO(0, 0, -RotSpeed);
                return false;
            }
            return false;
        } else {
            driveTrain.GO(0, 0, 0);
            ActionHelpers.action("Bot Target Angle Reached");
            return true;
        }
    }

    // TODO pass in motor to make fully parametric
    public boolean armRotateShoulder(double targetY_Rad, double armCurrent_Rad) {
        ActionHelpers.action("Rotating to " + targetY_Rad);

        double currentAngleY = armCurrent_Rad;

        // TODO config
        double tolerance = 0.1;
        double ySpeed = 0.70;
        double ySpeed_Rev = -0.15;

        if (Math.abs(targetY_Rad - currentAngleY) > tolerance) {
            if (currentAngleY > targetY_Rad) {
                //shouldeMotor.tankDrive(ySpeed, -ySpeed);
                return false;
            }
            if (currentAngleY < targetY_Rad) {
                //shouldeMotor.tankDrive(-ySpeed, ySpeed);
                return false;
            }
            return false;
        } else {
            //shouldeMotor.tankDrive(0, 0);
            ActionHelpers.action("Arm Target Angle Reached");
            return true;
        }
    }

    // TODO pass in motor to make fully parametric, make limitSwitch Boolean
    public boolean armExtend(double targetExtend, double armCurrentExtenshion_Metres, DigitalInput limitSwitch) {
        ActionHelpers.action("Extending to " + targetExtend);

        double currentDistance = armCurrentExtenshion_Metres;

        // TODO config
        double tolerance = 0.05;
        double xSpeed = 0.90;
        double xSpeed_Rev = -0.15;

        if (Math.abs(targetExtend - currentDistance) > tolerance) {
            if (currentDistance < targetExtend) {
                armExtension.GO(xSpeed);
                return false;
            }
            else {
                if (!limitSwitch.get()) {
                    armExtension.GO(-xSpeed);
                    return false;
                } else if (limitSwitch.get()) {
                    armExtension.GO(0);
                    return true;
                }
                return false;
            }
        } else {
            armExtension.GO(0);
            ActionHelpers.action("Arm Target Length Reached");
            return true;
        }
    }

    //// Old Auto Subroutines

    /*
    public void autoBalance(double targetAngle_Deg, double botPitch_Deg, PIDController balancePid) {
        double outputPitch;
        double tolerance = 5;

        if (Math.abs(targetAngle_Deg - botPitch_Deg) > tolerance) {
            outputPitch = balancePid.calculate(botPitch_Deg, targetAngle_Deg);
        } else {
            outputPitch = 0;
        }
        driveTrain.GO(outputPitch, 0, 0);
    }

    public void autoMidBalance() {
        if (timerInterval_Auto(0, 2.5)) {
            driveSwerve_EncoderIf_turnOnSpot(Math.PI - 0.01); // face nodes
            System.out.println("turning");
            frontLeftDrive.setSelectedSensorPosition(0);
        } else if (timerInterval_Auto(2.51, 3)) {
            driveSwerve_EncoderIf_FwdAndBwd(0.21); // drive forwards to thingy
            // swerveDrive(-0.2, 0, 0);
            System.out.println("forward");
        } else if (timerInterval_Auto(3.01, 6)) {
            swerveDrive(0, 0, 0);
            armRotate_encoderIf_upAndDown(-1.80); // lift arm
            System.out.println("arm up");
        } else if (timerInterval_Auto(6.01, 8)) {
            armExtend_encoderIf_outAndIn(0.75); // extend arm
            System.out.println("extend");
        } else if (timerInterval_Auto(8.01, 8.5)) {
            dSolenoidClaw.set(Value.kForward); // open claw
            System.out.println("drop");
        } else if (timerInterval_Auto(8.51, 10)) {
            armExtend_encoderIf_outAndIn(0); // retract
            dSolenoidClaw.set(Value.kReverse); // close claw
            System.out.println("arm down");
        } else if (timerInterval_Auto(10.01, 12)) {
            armRotate_encoderIf_upAndDown(-0.1); // lower arm
            System.out.println("retract");
        } else if (timerInterval_Auto(12.01, 15)) {
            driveSwerve_EncoderIf_FwdAndBwd(-2.3); // drive backwards past line
            System.out.println("back it up");
        } else { // STOP!!!
            swerveDrive(0, 0, 0);
            armRotate.tankDrive(0, 0);
            armTalonExtenstion.set(0);
            System.out.println("stop");
        }
    }

    public void autoTopAndBottom() {
        if (timerInterval_Auto(0, 2)) {
            driveSwerve_EncoderIf_turnOnSpot(Math.PI - 0.01); // face nodes
            System.out.println("turning");
        } else if (timerInterval_Auto(2.01, 3.50)) {
            driveSwerve_EncoderIf_FwdAndBwd(0.21); // drive forwards to thingy
            frontLeftDrive.setSelectedSensorPosition(0);
            // swerveDrive(-0.3, 0, 0);
            System.out.println("forward");
        } else if (timerInterval_Auto(3.51, 6)) {
            swerveDrive(0, 0, 0);
            armRotate_encoderIf_upAndDown(-1.80); // lift arm
            System.out.println("arm up");
        } else if (timerInterval_Auto(6.01, 8)) {
            armExtend_encoderIf_outAndIn(0.75); // extend arm
            System.out.println("extend");
        } else if (timerInterval_Auto(8.01, 8.5)) {
            dSolenoidClaw.set(Value.kForward); // open claw
            System.out.println("drop");
        } else if (timerInterval_Auto(8.51, 10)) {
            armExtend_encoderIf_outAndIn(0); // retract
            dSolenoidClaw.set(Value.kReverse); // close claw
            System.out.println("arm down");
        } else if (timerInterval_Auto(10.01, 12)) {
            armRotate_encoderIf_upAndDown(-0.1); // lower arm
            System.out.println("retract");
        } else if (timerInterval_Auto(12.01, 15)) {
            driveSwerve_EncoderIf_FwdAndBwd(-4.3); // drive backwards past line
            System.out.println("back it up");
        } else { // STOP!!!
            swerveDrive(0, 0, 0);
            armRotate.tankDrive(0, 0);
            armTalonExtenstion.set(0);
            System.out.println("stop");
        }
    }

    public boolean timerInterval_Auto(double min, double max) {
        if (timerAuto.get() > min && timerAuto.get() < max) {
            return true;
        } else {
            return false;
        }
    }
     */
}
