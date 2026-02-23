package frc.robot.autos.common;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.event.NetworkBooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DashboardConstants;
import frc.robot.RobotState;
import frc.robot.autos.modes.LeftNZRightClimb;

public class AutoFactory {
    // private final Supplier<Auto> autoSupplier = () -> Dashboard.getInstance().getAuto();
    // private final Supplier<Auto> autoSupplier;
    private Auto currentAuto;
    private AutoBase compiledAuto;
    private boolean autoCompiled;

    private boolean isRedAlliance = RobotState.getInstance().isRedAlliance();
    private static AutoFactory INSTANCE;
    public static AutoFactory getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new AutoFactory();
        }
        return INSTANCE;
    }
    private AutoFactory() {};

    public boolean recompileNeeded() {
        return autoSupplier.get() != currentAuto
                || isRedAlliance == !RobotState.getInstance().isRedAlliance();
    }

    public void recompile() {
        isRedAlliance = RobotState.getInstance().isRedAlliance();
        // update auto
        autoCompiled = false;
        currentAuto = autoSupplier.get();
        if (currentAuto == null) {
            currentAuto = Auto.NO_AUTO;
        }
        compiledAuto = currentAuto.getInstance();
        if (compiledAuto == null) {
        } else {
            compiledAuto.init(); // starts?
        }
        autoCompiled = true;
    }

    public static enum Auto {
        NO_AUTO(null),
        LEFT_NZ_RIGHT_CLIMB(LeftNZRightClimb.class);

        private final Class<? extends AutoBase> autoClass;

        private Auto(Class<? extends AutoBase> autoClass) {
            this.autoClass = autoClass;
        }

        public AutoBase getInstance() {
            if (autoClass != null) {
                try {
                    return autoClass.getConstructor().newInstance();

                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            return null;
        }
    }
}
