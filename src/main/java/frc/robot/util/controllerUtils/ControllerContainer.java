package frc.robot.util.controllerUtils;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.constants.OIConstants;

public class ControllerContainer {
    private final Controller[] controllers;

    public ControllerContainer() {
        controllers = new Controller[OIConstants.CONTROLLER_COUNT];

        OIConstants.CONTROLLERS.forEach((port, type) -> {
            switch (type) {
                case XBOX:
                    controllers[port] = new CustomXboxController(port);
                    break;
            }
        });
    }


    private static class CustomXboxController extends Controller {
        public CustomXboxController(int port) {
            super(port);
        }

        @Override
        public double getLeftY() {
            return -super.getRawAxis(XboxController.Axis.kLeftY.value);
        }

        @Override
        public double getLeftX() {
            return super.getRawAxis(XboxController.Axis.kLeftX.value);
        }

        @Override
        public double getRightY() {
            return -super.getRawAxis(XboxController.Axis.kRightY.value);
        }

        @Override
        public double getRightX() {
            return super.getRawAxis(XboxController.Axis.kRightX.value);
        }
    }
}