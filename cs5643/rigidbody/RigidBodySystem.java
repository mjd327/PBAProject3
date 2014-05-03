package cs5643.rigidbody;

import java.util.*;
import javax.vecmath.*;
import javax.media.opengl.*;


/**
 * Maintains a dynamic list of RigidBody objects, and provides access
 * to their state for numerical integration of dynamics and collision
 * processing.
 * 
 * @author Doug James, March 2007.
 */
public class RigidBodySystem 
{
    /** Current simulation time. */
    double time = 0;

    /** List of RigidBody objects. */
    HashSet<RigidBody>   bodies = new HashSet<RigidBody>();

    /** List of Force objects. */
    ArrayList<Force>      F = new ArrayList<Force>();

    CollisionProcessor collisionProcessor = null;
    boolean processCollisions = true;

    //ArrayList<Constraint> C = new ArrayList<Constraint>();

    /** Basic constructor. */
    public RigidBodySystem() {  }

    /** Adds a force object (until removed) */
    public synchronized void addForce(Force f) {
 	F.add(f);
    }

    /** Useful for removing temporary forces, such as user-interaction
     * spring forces. */
    public synchronized void removeForce(Force f) {
 	F.remove(f);
    }

    /** Fragile reference to rigid bodies.  */
    Collection<RigidBody> getRigidBodies() { 
	return bodies;
    }

    /** Number of rigid bodies. */
    public int getNBodies() { return bodies.size(); }

    /** Picks body based on some criteria, or null if none picked.  */
    public RigidBody pickBody(Point2d p)
    {
	double    pickDist = Double.MAX_VALUE;
	RigidBody pick     = null;
	for(RigidBody body : bodies) {

	    if(body.isPinned()) continue;

	    double dist = body.getPosition().distance(p);
	    if(dist < pickDist) {
		pickDist = dist;
		pick     = body;
	    }
	}
	return pick;
    }

    /** Adds the RigidBody to the system, and invalidates the existing
     * CollisionProcessor. */
    public synchronized void add(RigidBody rb) 
    {
	bodies.add(rb);

	/// INVALIDATE CollisionProcessor (needs to be rebuilt at timestep):
	collisionProcessor = null;
    } 


    /** Removes the RigidBody from the system, and invalidates the
     * existing CollisionProcessor. */
    public synchronized void remove(RigidBody rb) 
    {
	bodies.remove(rb);

	/// INVALIDATE CollisionProcessor (needs to be rebuilt at timestep):
	collisionProcessor = null;
    } 

    /** Moves all rigidbodys to undeformed/materials positions, and
     * sets all velocities to zero. Synchronized to avoid problems
     * with simultaneous calls to advanceTime(). */
    public synchronized void reset()
    {
	for(RigidBody rb : bodies)  {
	    rb.reset();
	    //rb.setHighlight(false);
	}
	time = 0;
    }

    /** Applies some random accelerations to the objects (modify as
     * desired). */
    public void jiggle() 
    {
	jiggle = true;
    }

    private boolean jiggle = false;

    private void applyJiggle()
    {
	Vector2d f   = new Vector2d();//-->nonzero
	double   tau = 0;
	double   amp = 10000;///BUG
	for(RigidBody body : bodies) {
	    double  m = body.getMass();
	    double  I = body.getMassAngular();
	    f.x = amp * m * (Math.random() - 0.5);
	    f.y = amp * m * (Math.random() - 0.5);
	    tau = 10 * amp * I * (Math.random() - 0.5);
	    body.applyWrenchW(f, tau);
	}
    }

    /**
     * Incomplete/Debugging integrator implementation. 
     * 
     * TODO: Modify this function to implement the integrator based on
     * the velocity-level complementarity constraint solver.
     */
    public synchronized void advanceTime(double dt)
    {
	/// NOTE: Clear force accumulators: already done after time step in RigidBody

 	{/// Gather forces: (TODO)

	    if(jiggle) {
		applyJiggle();
		jiggle = false;
	    }

  	    for(Force force : F)   force.applyForce();

  	    // GRAVITY + CHEAP DAMPING:
	    Vector2d f   = new Vector2d();//-->nonzero
	    double   tau = 0;
  	    for(RigidBody body : bodies) {
		f.x = f.y = 0;
		double   m = body.getMass();
		double   I = body.getMassAngular();

		/// DAMPING:
		double C = 0.1;
		Utils.acc(f,  -C * m, body.getVelocityLinear());//linear damping
		tau = - C/5. * I * body.getVelocityAngular();//angular damping

		f.y -= m * 10;//gravity

		body.applyWrenchW(f, tau);
	    }
 	}

	/// RESOLVE COLLISIONS!
	{
	    if(collisionProcessor == null) {//BUILD
		collisionProcessor = new CollisionProcessor(bodies);
	    }
	    if(processCollisions) collisionProcessor.processCollisions();
	}

	/// "ADVANCE TIME" SHOULD BE COMBINED WITH CONTACT SOLVER ...
	for(RigidBody body : bodies) {
	    body.advanceTime(dt);
	}

	time += dt;
    }

    /** Enables/disables collision processing. */
    public void setProcessCollisions(boolean enable)
    {
	processCollisions = enable;
    }
    /** Returns true if collision processing is enabled, and false
     * otherwise. */
    public boolean getProcessCollisions()
    {
	return processCollisions;
    }


    /**
     * Displays RigidBody and Force objects.
     */
    public synchronized void display(GL2 gl) 
    {
	for(RigidBody body : bodies) {
	    body.display(gl);
	}

 	for(Force force : F) {
 	    force.display(gl);
 	}
    }

}
