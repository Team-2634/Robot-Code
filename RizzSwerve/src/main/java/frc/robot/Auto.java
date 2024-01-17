package frc.robot;

public class Auto {
    AutoHelper autoHelper;
    public Auto(Driver driver, Shooter shooter, Climber climber) {
        this.autoHelper = new AutoHelper(driver, shooter, climber)
    }

    void restartAutoTimer() {
        timerAuto.reset();
        timerAuto.start();    
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
    
    




}
