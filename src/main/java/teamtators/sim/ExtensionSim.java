package teamtators.sim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class ExtensionSim extends LinearSystemSim<N2, N1, N1> {
    private DCMotor gearbox;
    private double mass;
    private double minLength;
    private double maxLength;
    private double angle;
    private boolean gravity;
    private double gravityAngle = -Math.PI / 2;

    // stuff to help with wrist simulation
    private Matrix<N2, N1> xdot = VecBuilder.fill(0, 0);

    public ExtensionSim(DCMotor gearbox, double mass, double minLength, double maxLength, double angle, boolean gravity) {
        super(LinearSystemId.createElevatorSystem(gearbox, mass, .02, 1./10), null);
        this.gearbox = gearbox;
        this.mass = mass;
        this.minLength = minLength;
        this.maxLength = maxLength;
        this.angle = angle;
        this.gravity = gravity;
    }

    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
        Matrix<N2, N1> updatedXhat = NumericalIntegration.rkdp((Matrix<N2, N1> x, Matrix<N1, N1> _u) -> {
            Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
            if (gravity) {
                xdot = xdot.plus(VecBuilder.fill(0, 9.81 * Math.cos(angle - gravityAngle)));
            }

            // hacked together fix to get the output equation
            this.xdot = xdot;

            return xdot;
        }, currentXhat, u, dtSeconds);

        // We check for collision after updating xhat
        if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(minLength, 0);
        }
        if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(maxLength, 0);
        }
        return updatedXhat;
    }

    public boolean wouldHitLowerLimit(double currentExtension) {
        return currentExtension <= this.minLength;
    }


    public boolean wouldHitUpperLimit(double currentAngleRads) {
        return currentAngleRads >= this.maxLength;
    }

    public boolean hasHitLowerLimit() {
        return getOutput(0) <= this.minLength;
    }

    public boolean hasHitUpperLimit() {
        return getOutput(0) >= this.maxLength;
    }


    public void setInputVoltage(double volts) {
        setInput(volts);
    }

    public double getTotalLength() {
        return getOutput(0);
    }

    public double getExtension() {
        return getTotalLength() - minLength;
    }

    public double getAccel() {
        return xdot.get(1, 0);
    }

    public static void main(String[] args) {
        double angle = -Math.PI / 2;
        ExtensionSim sim = new ExtensionSim(DCMotor.getNEO(1), 1, 1, 2, angle, true);
        sim.setInputVoltage(0);
        for (int i = 0; i < 100; i++) {
            sim.update(0.02);
//            System.out.println("("+ Math.cos(angle) * sim.getOutput(0) +","+ Math.sin(angle) * sim.getOutput(0) + ")");
            System.out.println("(" + 0 + "," + Math.sin(angle) * sim.getTotalLength() + ")");

        }
    }
}
