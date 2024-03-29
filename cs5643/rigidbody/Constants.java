package cs5643.rigidbody;

/**
 * Feel free to put your constants and parameters here. 
 *
 * @author Doug James, March 2007.
 */
public interface Constants
{
    /** Stiffness of mouse-force spring. */
    public static final double STIFFNESS_STRETCH_DENSITY = 500.;

    /** Stiffness of penalty contact spring force. */
    public static final double CONTACT_STIFFNESS = 200000.;
    
    /** Number of paths generated for each stochastic. */ 
    public static final int NUM_PATHS = 50; 
    
    /** The tolerance for how far a mouse click can be from a path */ 
    public static final double MOUSE_TOLERANCE = .01;
    
    public static final double epsilon = .0001;
}
