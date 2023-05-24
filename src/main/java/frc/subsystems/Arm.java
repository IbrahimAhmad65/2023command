package frc.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import teamtators.sim.PinkarmSim;


public class Arm extends SubsystemBase {
    private PinkarmSim pinkarmSim;

    public static class Constants {
        public static final int armRotationMotorID = 1;
        public static final int armExtensionMotorID = 2;
        public static boolean sim = SimWriter.sim;
        public static double armRotationError = 1./180 * Math.PI;
        public static double armExtensionError = .04;
    }

    private PIDController simExtensionPID;
    private PIDController simRotationPID;

    private CANSparkMax armRotationMotor;
    private CANSparkMax armExtensionMotor;
    private SparkMaxPIDController armRotationPID;
    private SparkMaxPIDController armExtensionPID;
    private double targetAngle = -Math.PI / 2;
    private double targetLength = .8;

    public Arm() {
        super();
        pinkarmSim = new PinkarmSim();

        if (Constants.sim){
            simExtensionPID = new PIDController(7,0,0);
            simRotationPID = new PIDController(8,0,0);
        } else {
            armExtensionMotor = new CANSparkMax(Constants.armExtensionMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
            armRotationMotor = new CANSparkMax(Constants.armRotationMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
            armExtensionPID = armExtensionMotor.getPIDController();
            armRotationPID = armRotationMotor.getPIDController();
        }
    }

    @Override
    public void periodic() {
        if(Constants.sim){
            pinkarmSim.setVoltages(simRotationPID.calculate(pinkarmSim.getAngleRads(), targetAngle), simExtensionPID.calculate(pinkarmSim.getLength(), targetLength));
            pinkarmSim.periodic();
        }
    }


    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public void setTargetLength(double targetLength) {
        this.targetLength = targetLength;
    }

    public boolean isAtTargetAngle() {
        return Math.abs(pinkarmSim.getAngleRads() - targetAngle) < Constants.armRotationError;
    }

    public boolean isAtTargetLength() {
        return Math.abs(pinkarmSim.getLength() - targetLength) < Constants.armExtensionError;
    }

    public double getCurrentLength() {
        return pinkarmSim.getLength();
    }
    public double getCurrentAngle() {
        return pinkarmSim.getAngleRads();
    }

    public double getTargetLength() {
        return targetLength;
    }
    public double getTargetAngle() {
        return targetAngle;
    }

    public PinkarmSim getDynamicsSim() {
        return pinkarmSim;
    }
}
