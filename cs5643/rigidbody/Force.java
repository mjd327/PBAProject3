package cs5643.rigidbody;

import javax.media.opengl.*;

/**
 * RigidBody system force.
 * 
 * @author Doug James, January 2007
 */
public interface Force  
{
    /** 
     * Causes force to be applied to affected bodies.
     */
    public void applyForce();

    /** Display any instructive force information, e.g., connecting spring. */
    public void display(GL2 gl);

    /** Reference to the system this force affects. */
    public RigidBodySystem getSystem();
}
