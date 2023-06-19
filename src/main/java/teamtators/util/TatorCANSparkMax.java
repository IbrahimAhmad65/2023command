package teamtators.util;

import com.revrobotics.*;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class TatorCANSparkMax {

    private CANSparkMax canSparkMax;
    private TatorCANPID canPID;
    private RelativeEncoder canEncoder;

    public TatorCANSparkMax(int deviceId, CANSparkMaxLowLevel.MotorType type) {
        canSparkMax = new CANSparkMax(deviceId, type);
        this.canPID = new TatorCANPID(canSparkMax);
        this.canEncoder = canSparkMax.getEncoder();
    }

    public void initForwardLimitSwitch(){
        canSparkMax.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
    }
    public void initForwardReverseSwitch(){
        canSparkMax.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
    }

    public SparkMaxLimitSwitch getForwardLimitSwitch(){
        return canSparkMax.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    }

    public SparkMaxLimitSwitch getReverseLimitSwitch(){
        return canSparkMax.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    }

    public MotorController getMotorController() {
        return canSparkMax;
    }

    public TatorCANPID getCanPID() {
        return canPID;
    }

    public void setVelocity(double value) {
        canPID.setReference(value, CANSparkMax.ControlType.kVelocity);
    }

    public void setPercentOutput(double value) {
        canSparkMax.set(value);
    }

    public void setPosition(double value){
        canPID.setPosition(value);
    }

    public CANSparkMax getCanSparkMax() {
        return canSparkMax;
    }

    public void setSmartMotion( double maxVel, double minVel, double maxAcc, double allowedErr ) {
        canPID.setSmartMotion(maxVel, minVel, maxAcc, allowedErr );
    }

    public void setReference( double setPoint ) {
        canPID.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion ) ;
    }

    public void setEncoderPosition(double value){
        System.out.println(value);
        canEncoder.setPosition(value);
    }

    // units must be checked
    public double getVelocity() {
        return canEncoder.getVelocity();
    }

    public double getPosition() {
        return canEncoder.getPosition();
    }

    public void stop() {
        canSparkMax.stopMotor();
    }

    public void setInverted(boolean inverted) {
        canSparkMax.setInverted(inverted);
    }

    public RelativeEncoder getEncoder(){
        return canEncoder;
    }
    public void configurePID(double p, double i, double d, double iZone, double f) {
        canPID.setPIDF(p, i, d, f);
        canPID.setIZone(iZone);
    }


}

