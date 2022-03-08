package frc.bionicpolars.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private enum State {
        RETRACTING,
        ROTATING,
        EXTENDING,
        IDLE
    };
    // Hard limits
    private static final double EXTENSION_MAX = 28.0,
            EXTENSION_MIN = -0.5,
            ANGLE_MAX = 80.0,
            ANGLE_MIN = -4.0,
        /*
         * Battery box constraint
         *
         * The arm must be atleast EXTENSION_MIN meters from the bottom
         * of travel when between ANGLE_MIN and ANGLE_MAX.
         */
            BATTERY_EXTENSION_MIN = 8.5,
            BATTERY_ANGLE_MIN = 3.0,
            BATTERY_ANGLE_MAX = 32.0,
        /*
         * Wall constraint
         *
         * The climber must be not extend past 16 inches from the end of the frame.
         */
            // Offset from the back of the frame to the rotation point
            FRAME_BACK_OFFSET = 26.5,
            // Max extension past the frame (G107 & R105)
            FRAME_EXTENSION_MAX = 16,
            // Safety margin from the limits
            SAFETY_MARGIN = 4.0,
            // Offset of arm from rotation point
            ARM_OFFSET = 2.5,
            // Width of the arn
            ARM_WIDTH = 1.0,
            // Length from the rotation point to the bottom end of the arm
            ARM_BACKWARD_LENGTH = 40.0,
        /*
         * Ceiling constraint
         *
         * The climber must be not extend past 5 foot 6 inches from the floor.
         */
            // Max height (G106)
            HEIGHT_MAXIMUM = 66.0,
            // Offset from the floor to the rotation point
            HEIGHT_OFFSET = 40.0,
            // Width of the arm plus hook
            HOOK_WIDTH = 4.0,
            // Length from the rotation point to the top end of the arm
            ARM_FORWARD_LENGTH = 11.0,
            // Next step tolerance
            STEP_TOLERANCE_ANGLE = 2.5,
            STEP_TOLERANCE_EXTENSION = 0.5;
    private ClimberExtend extend = new ClimberExtend();
    private ClimberRotate rotate = new ClimberRotate();
    private State state = State.IDLE;
    private double targetAngle = 0.0;
    private double targetExtension = 0.0;


    public double getExtension() {
        return extend.get();
    }

    public boolean setExtension(double extension) {
        return set(extension, getAngle());
    }

    public double getAngle() {
        return rotate.get();
    }

    public boolean setAngle(double angle) {
        return set(getExtension(), angle);
    }

    public boolean set(double extension, double angle) {
        boolean result = false;

        // Hard extension limits
        if(extension > EXTENSION_MAX) {
            extension = EXTENSION_MAX;
            System.out.println("Climber: constrained extension to: "
                    + extension + "m (hard)");
            result = true;
        }
        else if(extension < EXTENSION_MIN) {
            extension = EXTENSION_MIN;
            System.out.println("Climber: constrained extension to: "
                    + extension + "m (hard)");
            result = true;
        }

        // Hard angle limits
        if(angle > ANGLE_MAX) {
            angle = ANGLE_MAX;
            System.out.println("Climber: constrained angle to: "
                    + angle + "deg (hard)");
            result = true;
        }
        else if(angle < ANGLE_MIN) {
            angle = ANGLE_MIN;
            System.out.println("Climber: constrained angle to: "
                    + angle + "deg (hard)");
            result = true;
        }

        // Handle the battery box constraint
        state = State.IDLE;
        double currentAngle = getAngle();
        // Target is below the battery box maximum, and the current is above
        // aka transitioning into the battery box
        if(angle < BATTERY_ANGLE_MAX
                && currentAngle > BATTERY_ANGLE_MAX) {
            // If the target is below the minimum, setup a next step to extend again
            if(angle < BATTERY_ANGLE_MIN) {
                System.out.println("Climber: setup next step to: "
                        + extension + "m (down)");
                targetExtension = extension;
                state = State.EXTENDING;
            }
            // Constrain the extension to more than the extension minimum
            if(extension < BATTERY_EXTENSION_MIN) {
                extension = BATTERY_EXTENSION_MIN;
                System.out.println("Climber: constrained extension to: "
                        + extension + "m (battery box constraint)");
                result = true;
            }
        }
        // Target is above the battery box minimum, and the current is below
        // aka transitioning into the battery box
        else if(angle > BATTERY_ANGLE_MIN
                && currentAngle < BATTERY_ANGLE_MIN) {
            // If the target is above the maximum, setup a next step to extend again
            if(angle > BATTERY_ANGLE_MAX) {
                System.out.println("Climber: setup next step to: "
                        + extension + "m (up)");
                targetExtension = extension;
                state = State.EXTENDING;
            }
            // Constrain the extension to less than the extension maximum
            if(extension > BATTERY_EXTENSION_MIN) {
                extension = BATTERY_EXTENSION_MIN;
                System.out.println("Climber: constrained extension to: "
                        + extension + "m (battery box constraint)");
                result = true;
            }
        }
        // Current is within the battery box, constrain the extension
        else if(currentAngle < BATTERY_ANGLE_MAX
                && currentAngle > BATTERY_ANGLE_MIN) {
            // Current is in the battery box
            if(extension < BATTERY_EXTENSION_MIN) {
                extension = BATTERY_EXTENSION_MIN;
                System.out.println("Climber: constrained extension to: "
                        + extension + "m (battery box constraint)");
                result = true;
            }
        }

        // Handle the wall constraint
        double minimum;
        /* minimum = k1 - ((k2 - v1) / v2)
        * k1: ARM_BACKWARD_LENGTH
        * k2: FRAME_BACK_OFFSET + FRAME_EXTENSION_MAX - SAFETY_MARGIN
        * v1: ARM_OFFSET + ARM_WIDTH * cosine(angle)
        * v2: sine(angle)
        */
        // k2
        minimum = (FRAME_BACK_OFFSET
                + FRAME_EXTENSION_MAX
                - SAFETY_MARGIN);
        // minus v1
        minimum -= (ARM_OFFSET + ARM_WIDTH)
                * Math.cos(Math.toRadians(angle));
        // divided by v2
        minimum /= Math.sin(Math.toRadians(angle));
        // k1 minus
        minimum = ARM_BACKWARD_LENGTH - minimum;
        if(extension < minimum) {
            extension = minimum;
            System.out.println("Climber: constrained extension to: "
                    + extension + "m (wall constraint)");
            result = true;
        }

        // Handle the ceiling constraint
        double maximum;
        /* maximum = ((k1 - v1) / v2) - k2
        * k1: HEIGHT_MAXIMUM - HEIGHT_OFFSET
        * v1: ARM_OFFSET + HOOK_WIDTH * sine(angle)
        * v2: cosine(angle)
        * k2: ARM_FORWARD_LENGTH
        */
        // k1
        maximum = (HEIGHT_MAXIMUM
                - HEIGHT_OFFSET);
        // minus v1
        maximum -= (ARM_OFFSET + HOOK_WIDTH)
                * Math.sin(Math.toRadians(angle));
        // divided by v2
        maximum /= Math.cos(Math.toRadians(angle));
        // minus k2
        maximum -= ARM_FORWARD_LENGTH;
        if(extension > maximum) {
            extension = maximum;
            System.out.println("Climber: constrained extension to: "
                    + extension + "m (ceiling constraint)");
            result = true;
        }

        extend.set(extension);
        rotate.set(angle);
        return result;
    }

    public void zeroEncoders() {
        extend.zero();
        rotate.zero();
    }

    public void periodic() {
        switch(state) {
            case RETRACTING:
                if(BATTERY_EXTENSION_MIN - extend.getActual()
                        < STEP_TOLERANCE_EXTENSION) {
                    setAngle(targetAngle);
                    state = State.ROTATING;
                }
                break;
            case ROTATING:
                if(Math.abs(targetAngle - getAngle())
                        < STEP_TOLERANCE_ANGLE) {
                    setExtension(targetExtension);
                    state = State.EXTENDING;
                }
                break;
            case EXTENDING:
                if(Math.abs(targetExtension - extend.getActual())
                        < STEP_TOLERANCE_EXTENSION) {
                    state = State.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }
}
