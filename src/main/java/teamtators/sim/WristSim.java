package teamtators.sim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class WristSim extends LinearSystemSim<N2, N1, N2> {

    private double minAngle;
    private double maxAngle;
    private double width;
    private double mass;
    private boolean gravity;

    private PinkarmSim armSim;

    /*
     * Creates a simulation of a wrist
     *
     * @param gearbox the motor powering the wrist rotation
     * @param minAngle the smallest angle the wrist can rotate to
     * @param maxAngle the largest angle the wrist can rotate to
     */
    public WristSim(DCMotor gearbox, double minAngle, double maxAngle) {
        super(LinearSystemId.createDCMotorSystem(gearbox,.1,1./8), null);
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }

    /*
     * Creates a simulation of a wrist
     *
     * @param gearbox the motor powering the wrist rotation
     * @param minAngle the smallest angle the wrist can rotate to
     * @param maxAngle the largest angle the wrist can rotate to
     * @param mass the mass of the wrist IN KILOGRAMS
     * @param extensionSim an ExtensionSim representative of the extending arm
     * @param width the length in meters of the claw
     */
    public WristSim(DCMotor gearbox, double minAngle, double maxAngle, double mass, PinkarmSim armSim, double width, boolean gravity) {
        super(LinearSystemId.createDCMotorSystem(gearbox,.1,1./8), null);
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.armSim = armSim;
        this.width = width;
        this.mass = mass;
        this.gravity = gravity;
    }

    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
        Matrix<N2, N1> updatedXhat = NumericalIntegration.rkdp((Matrix<N2, N1> x, Matrix<N1, N1> _u) -> {
            Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
            // account for the torque applied to the wrist by the arm
            // NOTE width may be false, since center of mass is likely not directly in the middle of the object
            double forceByArm = (width / 2) * PinkarmSim.ArmConfig.mass2 * armSim.getExtensionAccel() * Math.cos(getAngle());
            double inertia = 1;
            if(gravity) {
                double forceByGravity = (width / 2) * mass * Math.sin(x.get(0, 0) + (Math.PI/2)) * -9.81;
                // FIXME find an actual equation for inertia, since this is completely made up
                xdot = xdot.plus(VecBuilder.fill(0, (forceByArm + forceByGravity) / inertia));
            } else {
                xdot = xdot.plus(VecBuilder.fill(0, forceByArm / inertia));
            }
            return xdot;
        }, currentXhat, u, dtSeconds);

        // We check for collision after updating xhat
        if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(minAngle, 0);
        }
        if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(maxAngle, 0);
        }
        return updatedXhat;
    }

    public boolean wouldHitLowerLimit(double currentExtension) {
        return currentExtension <= this.minAngle;
    }


    public boolean wouldHitUpperLimit(double currentAngleRads) {
        return currentAngleRads >= this.maxAngle;
    }

    public boolean hasHitLowerLimit() {
        return getOutput(0) <= this.minAngle;
    }

    public boolean hasHitUpperLimit() {
        return getOutput(0) >= this.maxAngle;
    }


    public void setInputVoltage(double volts) {
        setInput(volts);
    }

    public double getAngle() {
        return getOutput(0);
    }

    public String getDataToWrite() {
        return getAngle() + ",";
    }
}
