package teamtators.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

public class TatorCANPID {
    private SparkMaxPIDController device;
    public TatorCANPID(CANSparkMax device) {
        this.device = device.getPIDController();
    }
    
    public void setP(double p){
        device.setP(p);
    }
    public void setI(double I){
        device.setI(I);
    }
    public void setD(double d){
        device.setD(d);
    }

    public void setIZone(double iZone){
        device.setIZone(iZone);
    }

    public void setF(double f){
        device.setFF(f);
    }

    public void setSmartMotion( double maxVel, double minVel, double maxAcc, double allowedErr ) {
        device.setSmartMotionMaxVelocity(maxVel, 0) ;
        device.setSmartMotionMinOutputVelocity(minVel, 0);
        device.setSmartMotionMaxAccel(maxAcc, 0);
        device.setSmartMotionAllowedClosedLoopError(allowedErr, 0) ;
    }

    public void setPIDF(double p, double i, double d, double f){
        setP(p);
        setI(i);
        setD(d);
        setF(f);
    }

    public void setDevice(CANSparkMax device){
        this.device = device.getPIDController();
    }

    public void setReference(double value, CANSparkMax.ControlType controlType){
        device.setReference(value,controlType);
    }

    public void setVelocity(double value){
        device.setReference(value, CANSparkMax.ControlType.kVelocity);
    }

    public void setPosition(double value){
        device.setReference(value, CANSparkMax.ControlType.kPosition);
    }

    public SparkMaxPIDController getDevice() {
        return device;
    }
}