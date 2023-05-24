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


    public WristSim(DCMotor gearbox, double minAngle, double maxAngle) {
        super(LinearSystemId.createDCMotorSystem(gearbox,.1,1./8), null);
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }

    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
        Matrix<N2, N1> updatedXhat = NumericalIntegration.rkdp((Matrix<N2, N1> x, Matrix<N1, N1> _u) -> {
            Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
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


    public static void main(String[] args) {
        double angle = -Math.PI / 2;
        WristSim sim = new WristSim(DCMotor.getNEO(1), 0, Math.PI);
        sim.setInputVoltage(12);
        for (int i = 0; i < 100; i++) {
            sim.update(0.02);
//            System.out.println("("+ Math.cos(angle) * sim.getOutput(0) +","+ Math.sin(angle) * sim.getOutput(0) + ")");
            System.out.println("(" + ((double)i)/100 +"," + sim.getAngle() +")");

        }
    }
}
