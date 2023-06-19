package frc.subsystems;


import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.SwerveConstants;
import frc.robot.RobotContainer;
import teamtators.util.*;

import java.util.function.Consumer;

public class SwerveDrive extends SubsystemBase implements Consumer<ChassisSpeeds> {

    private SwerveModule[] moduleArray;
    private SwerveModule module0;           // The four swerve modules
    private SwerveModule module1;
    private SwerveModule module2;
    private SwerveModule module3;
    private TatorPigeon gyro;

    private NetworkTableEntry gyroTable = NetworkTableInstance.getDefault().getTable("visionTable").getEntry("newGyroAngle");//    private AHRS gyro;
    private NetworkTableEntry gyroRecal = NetworkTableInstance.getDefault().getTable("visionTable").getEntry("gyroRecal");//    private AHRS gyro;


    private final double PI = Math.PI;

    private boolean setAngleOverride;


    // PID Controllers

    private double desiredAngle;




    private double theta;
    private double currentAngle;

    private final double MOTOR_ROTATIONS_PER_MODULE_ROTATION;
    private final double MOTOR_ROTATIONS_PER_WHEEL_ROTATION;
    private final double WHEEL_CIRCUMFERENCE;

    private NetworkTableInstance inst;

    private final String tableKey = "PositionTable";
    private final String xVectorsKey = "XVectors";
    private final String yVectorsKey = "YVectors";
    private final String posInTableKey = "posInTable";
    private final String gyroKey = "gyroTheta";

    int counter = 0;

    private NetworkTableEntry xVectors;
    private NetworkTableEntry yVectors;
    private NetworkTableEntry posInTable;
    private NetworkTableEntry gyroTheta;

    private double[] posArray = new double[2];
    private double[] xVectorComps;
    private double[] yVectorComps;
    private ChassisSpeeds chassisSpeeds;

    private boolean lockVel;
    private double acceleration = .32;
    private double zoomyModeMultiple = .9;
    private SwerveDriveKinematics swerveDriveKinematics;

    //    private Vision vision;
    public SwerveDrive(){
        super();
        this.MOTOR_ROTATIONS_PER_WHEEL_ROTATION = SwerveConstants.SwerveModule.MOTOR_ROTATIONS_PER_WHEEL_ROTATION;
        this.MOTOR_ROTATIONS_PER_MODULE_ROTATION = SwerveConstants.SwerveModule.MOTOR_ROTATIONS_PER_MODULE_ROTATION;
        this.WHEEL_CIRCUMFERENCE = SwerveConstants.SwerveModule.WHEEL_CIRCUMFERENCE;



        Translation2d module0Pos = new Translation2d(-16, -13.5);
        Translation2d module1Pos = new Translation2d(16, -13.5);
        Translation2d module2Pos = new Translation2d(16, 13.5);
        Translation2d module3Pos = new Translation2d(-16, 13.5);
        swerveDriveKinematics = new SwerveDriveKinematics(module0Pos, module1Pos, module2Pos, module3Pos);


        inst = NetworkTableInstance.getDefault();
        xVectors = inst.getTable(tableKey).getEntry(xVectorsKey);
        yVectors = inst.getTable(tableKey).getEntry(yVectorsKey);
        posInTable = inst.getTable(tableKey).getEntry(posInTableKey);
        gyroTheta = inst.getTable(tableKey).getEntry(gyroKey);

        xVectorComps = new double[5];
        yVectorComps = new double[5];
        configure();
        gyro = new TatorPigeon(0, SwerveConstants.SwerveModule.canivoreBusName);
        gyro.zero();
        // Shuffleboard stuff:

    }


    public void doPeriodic() {
        gyroTable.setDouble(gyro.getYawContinuous());
    }








    public void updateModules() {
        var states = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        module0.setMotion(states[0]);
        module1.setMotion(states[1]);
        module2.setMotion(states[2]);
        module3.setMotion(states[3]);

    }


    public double rotationsToMeters(double rotations) {
        double speed = (rotations / MOTOR_ROTATIONS_PER_WHEEL_ROTATION) * WHEEL_CIRCUMFERENCE;
        return speed;
    }



    public double getDesiredAngle() {
        return desiredAngle;
    }

    public void enableOverride() {
        setAngleOverride = true;
    }





    public double getRotation() {
        return Math.toRadians(gyro.getYawD());
    }



    public void resetYaw() {
        gyro.zero();
        setAngleOverride = false;
    }

    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return swerveDriveKinematics;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }




    public void bumpPDown() {
        module0.setP(module0.getP() - .05);
        module1.setP(module1.getP() - .05);
        module2.setP(module2.getP() - .05);
        module3.setP(module3.getP() - .05);
    }

    public void bumpPUp() {
        module0.setP(module0.getP() + .05);
        module1.setP(module1.getP() + .05);
        module2.setP(module2.getP() + .05);
        module3.setP(module3.getP() + .05);
    }

    public void stop() {
        module0.stop();
        module1.stop();
        module2.stop();
        module3.stop();
    }









    public void printAllPositions() {
        module0.printPosition();
        module1.printPosition();
        module2.printPosition();
        module3.printPosition();
    }

    public void configure() {

        CANCoderWrapper encoder0 = new CANCoderWrapper(9, SwerveConstants.SwerveModule.canivoreBusName);
        CANCoderWrapper encoder1 = new CANCoderWrapper(10, SwerveConstants.SwerveModule.canivoreBusName);
        CANCoderWrapper encoder2 = new CANCoderWrapper(11, SwerveConstants.SwerveModule.canivoreBusName);
        CANCoderWrapper encoder3 = new CANCoderWrapper(12, SwerveConstants.SwerveModule.canivoreBusName);

        try {
            Thread.sleep(2000);
        } catch (Exception ignored) {

        }


        module0 = new SwerveModule(0, new TalonFXWrapper(1, SwerveConstants.SwerveModule.canivoreBusName), new TatorCANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless), encoder0);
        module1 = new SwerveModule(1, new TalonFXWrapper(2, SwerveConstants.SwerveModule.canivoreBusName), new TatorCANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless), encoder1);
        module3 = new SwerveModule(3, new TalonFXWrapper(3, SwerveConstants.SwerveModule.canivoreBusName), new TatorCANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless), encoder2);
        module2 = new SwerveModule(2, new TalonFXWrapper(4, SwerveConstants.SwerveModule.canivoreBusName), new TatorCANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless), encoder3);

        moduleArray = new SwerveModule[]{module0, module1, module2, module3};
    }

    public TatorPigeon getGyro() {
        return gyro;
    }


    public void reset() {

    }

    public String printName() {
        return ("SwerveDrive");
    }

    public void lockModules() {
        module0.lock();
        module1.lock();
        module2.lock();
        module3.lock();
    }

    public SwerveModule[] getSwerveModules(){
        return new SwerveModule[]{module0,module1,module2,module3};
    }
    public void unlockModules() {
        module0.unLock();
        module1.unLock();
        module2.unLock();
        module3.unLock();
    }



    @Override
    public void accept(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }
}