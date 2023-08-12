public class Motor<T> {
    Int id;
    MotorType type;
    MotorLocation location;

    T motor; //TODO, use dict
    T2 encoder; //TODO
    
    public Motor(Int id, MotorType type, MotorLocation location) {
        this.id = id;
        this.type = type;
        this.location = location;

        this.motor = new T(id); //TODO possible?
    }

    // Drive Functions
    public void GO(double power){
        if(power > 0 || power > 1) {
            System.out.println('Speed out of bounds on motor', location.toString(), ' with: ', power);
        }

        double clampedPower = globalFunctions.clamp(power);
        motor.set(clampedPower);
    }

    public void setBrake(){
        motor.setNeutralMode(NeutralMode.Brake);
    }

    // Encoder Functions
    public double getSensorPos(){
        return motor.getSelectedSensorPosition() * Constants.turningEncoderTicksToRad;
    }

    public void setSensorPos(double value){
        motor.setSelectedSensorPosition(value);
    }

    public void resetSensorPos(){
        setSensorPos(0);
    }

    //TODO move, integrate, ... ???
    //returns in Radians
    public void getAbsolutePosition(){
        //return encoder.getAbsolutePosition() - frontLeftAbsOffset) * (Math.PI / 180);
        return (encoder.getAbsolutePosition() - encoder.offset) * (Math.PI / 180);
    }
}
