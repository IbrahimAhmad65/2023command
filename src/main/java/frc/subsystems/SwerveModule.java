package frc.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import teamtators.util.*;
import frc.constants.*;
public class SwerveModule {

    /**
     * An enumeration for the control state of the module
     */
    public enum ModuleState {
        /**
         * The rotation PID is running and the module will try and move to the correct angle
         */
        Enabled,
        /**
         * The rotation PID is not running and the module can be moved by hand
         */
        Disabled,
        Locked
    }

    private ModuleState currentState = ModuleState.Enabled;

    //Config Values
    private final int moduleNumber;

    private final double MOTOR_ROTATIONS_PER_MODULE_ROTATION;
    private final double MOTOR_ROTATIONS_PER_WHEEL_ROTATION;
    private final double WHEEL_CIRCUMFERENCE;
    private boolean sim = true;
    private double distance;
    private double angle;
    private Timer timer;


    private final double[][] mainBotModulePositions = {{1,1}, {-1, 1}, {-1, -1}, {1, -1}};
    private final double[][] swerveBotModulePositions = {{-16, -13.5}, {16, -13.5}, {16, 13.5}, {-16, 13.5}};

    private final double[] directionFixOffset = {1, 1, 1, 1};
    private final double[] rotationFixOffset = {0, 0, 0, 0};

    private double[] mainBotEncoderOffsets = {0, 0, 0, 0};

    // These are the starting values
    private final double[] compBotEncoderOffsets = SwerveConstants.SwerveModule.SWERVE_MODULE_OFFSETS.clone();

    // Hardware
    private volatile CANCoderWrapper absoluteEncoder;     // Encoder used for checking rotation angles
    private TatorCANSparkMax rotationMotor;          // NEO550 is rotation motor
    private TalonFXWrapper movementMotor;      // Talon FX is the movement motor
    private RelativeEncoder rotationEncoder;
    private double direction;


    // Controllers
    private SwerveCANPIDRotationController canPID;

    // Vectors
    private Matrix<N2, N1> endVector = new Matrix<N2,N1>(Nat.N2(),Nat.N1());    // Vector used for moving the motors


    // Logger

    // Constants
    private double PI = Math.PI;
    private double TAU = 2 * PI;
    private double MAX_ERROR = .01;

    boolean mainBot = true;

    public SwerveModule(int module, TalonFXWrapper driveMotor, TatorCANSparkMax rotationMotor, CANCoderWrapper canCoder) {
        this.moduleNumber = module;
        driveMotor.setNeutralMode(NeutralMode.Brake);
        movementMotor = driveMotor;
        this.rotationMotor = rotationMotor;
         rotationMotor.setInverted(false);
         movementMotor.setInverted(true);
        this.absoluteEncoder = canCoder;
        this.MOTOR_ROTATIONS_PER_WHEEL_ROTATION = SwerveConstants.SwerveModule.MOTOR_ROTATIONS_PER_WHEEL_ROTATION;
        this.MOTOR_ROTATIONS_PER_MODULE_ROTATION = SwerveConstants.SwerveModule.MOTOR_ROTATIONS_PER_MODULE_ROTATION;
        this.WHEEL_CIRCUMFERENCE = SwerveConstants.SwerveModule.WHEEL_CIRCUMFERENCE;
        initialize();
        canCoder = null;
        timer = new Timer();
    }

    public void initialize() {
        // First, we need to apply the offset corrections
        mainBotEncoderOffsets[moduleNumber] = (rotationFixOffset[moduleNumber] + compBotEncoderOffsets[moduleNumber]) * directionFixOffset[moduleNumber];
        rotationEncoder = rotationMotor.getEncoder();
        canPID = new SwerveCANPIDRotationController(rotationMotor.getCanSparkMax(), MOTOR_ROTATIONS_PER_MODULE_ROTATION);
        canPID.setP(1);
        canPID.setD(.05);
        movementMotor.configurePID(.05, 0.0, 0.0, 1.0, 0.0 );
        calibrate();
    }

    public void calibrate() {
        // double[] offsets;
            // offsets = mainBotEncoderOffsets;
            // rotationVector.setXY(mainBotModulePositions[moduleNumber][0], mainBotModulePositions[moduleNumber][1]);
            // rotationVector.setTheta(rotationVector.getTheta() + (Math.PI / 2));
            // rotationVector.setMagnitude(1);

        // double currentValue = absoluteEncoder.getConvertedAbsolute() + offsets[moduleNumber];
        // System.out.println(currentValue);
        absoluteEncoder.setPosition(absoluteEncoder.getAbsolutePosition());
        System.out.println((absoluteEncoder.getAbsolutePosition()/360.0) * MOTOR_ROTATIONS_PER_MODULE_ROTATION);
        rotationEncoder.setPosition((absoluteEncoder.getAbsolutePosition()/360.0) * MOTOR_ROTATIONS_PER_MODULE_ROTATION);
    }

    public void stop() {
        canPID.setReference(rotationEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
        movementMotor.stop();
        rotationMotor.stop();
    }

    public void reset() {
        canPID.setReference(0, CANSparkMax.ControlType.kPosition);
    }


    public void setMotion(SwerveModuleState swerveModuleState){
        if(!timer.isRunning()){
            timer.start();
        } else {
            double t = timer.restart();
        }
        var mag = swerveModuleState.speedMetersPerSecond;
        var angle = swerveModuleState.angle.getRadians();
        this.angle = angle;
        this.distance += mag * timer.get();
        switch (currentState) {
            case Enabled:
                if (mag != 0) {
                    direction = canPID.setOptimizedPositionNew(angle);
                    movementMotor.setVelocity(mag  * direction * baseSpeed());

                } else {
                    canPID.setReference(rotationEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
                    // rotationMotor.stopMotor();
                    movementMotor.stop();
                }
                break;
            case Disabled:
                break;
            case Locked:
                direction = canPID.setOptimizedPositionNew(0);
                movementMotor.set(0);
                break;


        }
    }


    public SwerveModuleState getModuleState() {
        if (sim) {
            return new SwerveModuleState(distance, new Rotation2d(angle));
        }
        return new SwerveModuleState(rotationsToMeters(movementMotor.getPosition()), new Rotation2d(getCurrentAngle()));
    }

    public void printPosition() {
    }

    public double motorRotationsToRadians(double rotations) {
        return (rotations / MOTOR_ROTATIONS_PER_MODULE_ROTATION) * TAU;
    }

    public double rotationsToInches(double rotations) {
        return rotations / MOTOR_ROTATIONS_PER_WHEEL_ROTATION * WHEEL_CIRCUMFERENCE;
    }

    public static double degreesToModuleRotations(double degrees) {
        return (degrees / 360 * SwerveConstants.SwerveModule.MOTOR_ROTATIONS_PER_MODULE_ROTATION);
    }

    public double rotationsToMeters(double rotations) {
        double speed = (rotations / MOTOR_ROTATIONS_PER_WHEEL_ROTATION) * WHEEL_CIRCUMFERENCE;
        return speed;
    }

    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(rotationsToMeters(movementMotor.getPosition()),Rotation2d.fromRadians(getCurrentAngle()));
    }

    public double getCurrentAngle() {
        return (motorRotationsToRadians(rotationEncoder.getPosition()));
    }

    public void setP(double p) {
        canPID.setP(p);
    }

    public double getP() {
        return canPID.getP();
    }

    public double getDesiredAngleError() {
        return canPID.getDesiredModuleAngle() - getCurrentAngle();
    }

    public double getAngleError() {
        double var = Math.abs(canPID.getTargetAngle() % (2 * PI) - getCurrentAngle() % (2 * PI)) % (2 * PI);
        return Math.min(var, Math.abs(TAU - var));
    }


    public double IPMtoRPM(double IPM) {
        double wheelRotations = IPM / WHEEL_CIRCUMFERENCE;
        return wheelRotations * MOTOR_ROTATIONS_PER_WHEEL_ROTATION;
    }

    public boolean atTargetAngle(double maxError) {
        return getDesiredAngleError() < maxError;
    }

    public double getVelocity() {
        return rotationsToInches(movementMotor.getVelocity());
    }


    public static double baseSpeed(){
        return ( 4000);
    }

    @Override
    public String toString() {
        return "SwerveModule{" +
                ", rotationVectors=" + mainBotModulePositions[moduleNumber] +
                ", movementMotor=" + movementMotor.getVelocity() +
                ", rotationEncoder=" + rotationEncoder.getPosition() +
                ", endVector=" + endVector +
                ", MAX_ERROR=" + MAX_ERROR +
                '}';
    }

    public void lock(){
        currentState = ModuleState.Locked;
    }
    public void unLock(){
        currentState = ModuleState.Enabled;
    }


    public void configureCurrentRampAuto(){
        movementMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40,50,.001));
        movementMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40,50,.001));
    }

    public void configureCurrentRampTele(){
        movementMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 55,65,.001));
        movementMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40,50,.001));
    }


}