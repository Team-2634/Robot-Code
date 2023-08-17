/*
public class actions {

    //TODO this will be the list of basic auto actions, ex. DriveDist, TurnDeg, Balance...
    // In an additional ActionHandler(?) class we will have a series of ActionGroups(?) which are the assembled auto code ex. DrivePlaceTurnDrive
    //THe one to be used will be selected inside main 

    public void run(){




    }






    public boolean driveSwerve_EncoderIf_FwdAndBwd(double targetX) {
        targetX = -targetX;

        double currentDistanceX;
        currentDistanceX = encoderLeftFrontDriveDisplacement_Meteres;
        double outPutX = 0;

        double toleranc = 0.05;
        double xSpeed = 0.45;
        double xSpeed_Rev = -0.30;
        if (Math.abs(targetX - currentDistanceX) > toleranc) {
            if (currentDistanceX < targetX) {
                outPutX = xSpeed;
                swerveDrive(outPutX, 0, 0);
                System.out.println("behind target");
                return false;
            }
            if (currentDistanceX > targetX) {
                outPutX = -xSpeed;
                swerveDrive(outPutX, 0, 0);
                System.out.println("in front of target");
                return false;
            }
            return false;
        } else {
            swerveDrive(0, 0, 0);
            return true;
        }
    }

    public boolean driveSwerve_EncoderIf_turnOnSpot(double targetYaw_inRad) {
        double currentRoationYaw_inRad;
        currentRoationYaw_inRad = botYaw_angleRad;
        double outPutRad = 0;

        double tolerance = 0.2;
        double RotSpeed = 25; // rads per sec
        if (Math.abs(targetYaw_inRad - currentRoationYaw_inRad) > tolerance) {
            if (currentRoationYaw_inRad < targetYaw_inRad) {
                outPutRad = RotSpeed;
                swerveDrive(0, 0, outPutRad);
                return false;
            } else if (currentRoationYaw_inRad > targetYaw_inRad) {
                outPutRad = -RotSpeed;
                swerveDrive(0, 0, outPutRad);
                return false;
            }
            return false;
        } else {
            swerveDrive(0, 0, 0);
            return true;
        }
    }

    public boolean armRotate_encoderIf_upAndDown(double targetY) {

        double currentDistanceY;
        currentDistanceY = armRad_current;
        double outPutY = 0;

        double toleranc = 0.1;
        double ySpeed = 0.70;
        double ySpeed_Rev = -0.15;
        if (Math.abs(targetY - currentDistanceY) > toleranc) {
            if (currentDistanceY > targetY) {
                outPutY = -ySpeed;
                armRotate.tankDrive(-outPutY, outPutY);
                return false;
            }
            if (currentDistanceY < targetY) {
                outPutY = ySpeed;
                armRotate.tankDrive(-outPutY, outPutY);
                return false;
            }return false;
        } else {
            armRotate.tankDrive(0, 0);
            return true;
        }
    }

    public boolean armExtend_encoderIf_outAndIn(double targetExtend) {

        double currentDistance_Metres;
        currentDistance_Metres = extenstionEncoder_CurrentMetres;
        double outPut_prec = 0;

        double toleranc = 0.05;
        double xSpeed = 0.90;
        double xSpeed_Rev = -0.15;
        if (Math.abs(targetExtend - currentDistance_Metres) > toleranc) {
            if (currentDistance_Metres < targetExtend) {
                outPut_prec = xSpeed;
                armTalonExtenstion.set(outPut_prec);
                return false;
            }
            if (currentDistance_Metres > targetExtend) {
                if (!extendLimitSwitch.get()) {
                    outPut_prec = -xSpeed;
                    armTalonExtenstion.set(outPut_prec);
                    return false;
                } else if (extendLimitSwitch.get()) {
                    armTalonExtenstion.set(0);
                    return false;
                }
            }
        }
        else {
            armTalonExtenstion.set(0);
            return true;
        }
        return false;
    }

    public void autoBalance() {
        double outputPitch;
        double currentPitch;
        double targetAnglePitch = 0;
        double tolerance = 5;
        currentPitch = navxPitch_Deg;

        if (Math.abs(targetAnglePitch - currentPitch) > tolerance) {
            outputPitch = pidPitch.calculate(currentPitch, targetAnglePitch);
        } else {
            outputPitch = 0;
        }
        
        swerveDrive(outputPitch, 0, 0);
    }

    public void autoMidBalance() {
        if (timerInterval_Auto(0, 2.5)){
            driveSwerve_EncoderIf_turnOnSpot(Math.PI-0.01); // face nodes
            System.out.println("turning");
            frontLeftDrive.setSelectedSensorPosition(0);
        }else if (timerInterval_Auto(2.51, 3)){
            driveSwerve_EncoderIf_FwdAndBwd(0.21); //drive forwards to thingy
            //swerveDrive(-0.2, 0, 0);
            System.out.println("forward");
        }else if (timerInterval_Auto(3.01, 6)){
            swerveDrive(0,0,0);
            armRotate_encoderIf_upAndDown(-1.80); //lift arm
            System.out.println("arm up");
        }else if (timerInterval_Auto(6.01, 8)){
            armExtend_encoderIf_outAndIn(0.75); // extend arm
            System.out.println("extend");
        }else if (timerInterval_Auto(8.01, 8.5)){
            dSolenoidClaw.set(Value.kForward); //open claw
            System.out.println("drop");
        }else if (timerInterval_Auto(8.51, 10)){
            armExtend_encoderIf_outAndIn(0); //retract
            dSolenoidClaw.set(Value.kReverse); //close claw
            System.out.println("arm down");
        }else if (timerInterval_Auto(10.01, 12)){
            armRotate_encoderIf_upAndDown(-0.1); //lower arm
            System.out.println("retract");
        }else if (timerInterval_Auto(12.01, 15)){
            driveSwerve_EncoderIf_FwdAndBwd(-2.3); //drive backwards past line
            System.out.println("back it up");
        }else { //STOP!!!
            swerveDrive(0, 0, 0);
            armRotate.tankDrive(0, 0);      
            armTalonExtenstion.set(0);
            System.out.println("stop");
        }
    }

    public void autoTopAndBottom() {
        if (timerInterval_Auto(0, 2)){
            driveSwerve_EncoderIf_turnOnSpot(Math.PI-0.01); // face nodes
            System.out.println("turning");
        }else if (timerInterval_Auto(2.01, 3.50)){
            driveSwerve_EncoderIf_FwdAndBwd(0.21); //drive forwards to thingy
            frontLeftDrive.setSelectedSensorPosition(0);
            //swerveDrive(-0.3, 0, 0);
            System.out.println("forward");
        }else if (timerInterval_Auto(3.51, 6)){
            swerveDrive(0,0,0);
            armRotate_encoderIf_upAndDown(-1.80); //lift arm
            System.out.println("arm up");
        }else if (timerInterval_Auto(6.01, 8)){
            armExtend_encoderIf_outAndIn(0.75); // extend arm
            System.out.println("extend");
        }else if (timerInterval_Auto(8.01, 8.5)){
            dSolenoidClaw.set(Value.kForward); //open claw
            System.out.println("drop");
        }else if (timerInterval_Auto(8.51, 10)){
            armExtend_encoderIf_outAndIn(0); //retract
            dSolenoidClaw.set(Value.kReverse); //close claw
            System.out.println("arm down");
        }else if (timerInterval_Auto(10.01, 12)){
            armRotate_encoderIf_upAndDown(-0.1); //lower arm
            System.out.println("retract");
        }else if (timerInterval_Auto(12.01, 15)){
            driveSwerve_EncoderIf_FwdAndBwd(-4.3); //drive backwards past line
            System.out.println("back it up");
        }else { //STOP!!!
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
}
*/
