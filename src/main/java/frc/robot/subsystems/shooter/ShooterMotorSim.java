package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LoggedTalon.Follower.PhoenixTalonFollower;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXFlywheelSim;
import frc.robot.util.LoggedTalon.TalonInputs;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class ShooterMotorSim extends TalonFXFlywheelSim {

    private static final double SHOT_THRESHOLD_ROT_PER_S = 20.0;

    private final Timer bpsTimer = new Timer();
    private final SwerveDriveSimulation driveSimulation;

    /**
     * A simple flywheel sim representing a {@link LoggedTalonFX}
     *
     * <p>This sim is enhanced with CTRE's high fidelity simulation
     *
     * @param canID The motor's CAN ID
     * @param canBus The motor's CAN Bus
     * @param name The Motors Name. This <strong>MUST NOT</strong> be changed in replay.
     * @param J_KgMetersSquared The inertia of the system, in Kgm^2. See
     *     {@link LinearSystemId#createFlywheelSystem(DCMotor, double, double)}
     * @param gearReduction The gear reduction of the system. See {@link LinearSystemId#createFlywheelSystem(DCMotor,
     *     double, double)}
     * @param followers Followers, if any. Followers will share the same output as the leader. All followers are
     *     designed to be physically connected to the leader and as such their velocity and position are not accessible
     *     separately. The current number off followers <strong>MUST</strong> be passed into simulation and replay.
     */
    public ShooterMotorSim(
            int canID,
            CANBus canBus,
            String name,
            double J_KgMetersSquared,
            double gearReduction,
            SwerveDriveSimulation driveSimulation,
            PhoenixTalonFollower... followers) {
        super(canID, canBus, name, J_KgMetersSquared, gearReduction, followers);
        this.driveSimulation = driveSimulation;
    }

    @Override
    protected void simulationPeriodic(TalonInputs inputs) {
        super.simulationPeriodic(inputs);
        if (inputs.velocityRotPS > SHOT_THRESHOLD_ROT_PER_S) {
            bpsTimer.start();
            if (bpsTimer.advanceIfElapsed(1.0 / 4.0)) {
                SimulatedArena.getInstance()
                        .addGamePieceProjectile(new RebuiltFuelOnFly(
                                        driveSimulation
                                                .getSimulatedDriveTrainPose()
                                                .getTranslation(),
                                        new Translation2d(0.2, 0), // shooter offset from center
                                        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                        driveSimulation
                                                .getSimulatedDriveTrainPose()
                                                .getRotation(),
                                        Units.Meters.of(0.4), // initial height of the ball, in meters
                                        Units.MetersPerSecond.of(
                                                inputs.velocityRotPS * Math.PI * 0.05), // initial velocity, in m/s
                                        Units.Degrees.of(120)) // shooter angle
                                .withProjectileTrajectoryDisplayCallBack(
                                        (poses) -> Logger.recordOutput(
                                                "successfulShotsTrajectory", poses.toArray(Pose3d[]::new)),
                                        (poses) -> Logger.recordOutput(
                                                "missedShotsTrajectory", poses.toArray(Pose3d[]::new))));
            }
        } else {
            bpsTimer.stop();
            bpsTimer.reset();
        }
    }
}
