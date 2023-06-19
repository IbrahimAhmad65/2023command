package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import teamtators.sim.PinkarmSim;
import teamtators.sim.WristSim;

public class Wrist extends SubsystemBase {
    private double error = 0.04;
    private WristSim wristSim;
    private double targetAngle = 0;
    private DutyCycleEncoder encoder;
    private CANSparkMax motor;
    private final boolean sim = SimWriter.sim;
    private PIDController controller;

    public Wrist(PinkarmSim armSim) {
        super();
        if (sim) {
            controller = new PIDController(1,0,0);
            wristSim = new WristSim(DCMotor.getNeo550(1), 0, Math.PI, 5, armSim, 0.1);
        } else {
            encoder = new DutyCycleEncoder(0);
            motor = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
        }
    }

    @Override
    public void periodic() {
        if (sim){
            wristSim.setInputVoltage(controller.calculate(wristSim.getAngle(), targetAngle));
            wristSim.update(0.02);
            System.out.println(wristSim.getAngle());
        }
    }

    public WristSim getDynamicsSim() {
        return wristSim;
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public boolean isAtTargetAngle() {
        if (sim) {
            return Math.abs(wristSim.getAngle() - targetAngle) < error;
        } else {
            return Math.abs(encoder.get() - targetAngle) < error;
        }
    }
}
