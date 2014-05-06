package cs5643.rigidbody;

import java.util.*;
import javax.vecmath.*;
import javax.media.opengl.*;

/** 
 * Simple 2D rigid body with image- and sample-based geometric
 * representation.
 *
 * @author Doug James, March 2007.
 */
public class RigidBody
{
    /** Mass of object. */
    double   massLinear;

    /** Angular mass, I_zz, the scalar 2D version of the inertia tensor.  */
    double   massAngular;

    /** Position of center of mass (world frame)  */
    Point2d  x  = new Point2d();

    /** Initial position of center of mass (world frame)  */
    Point2d  x0 = new Point2d();

    /** Linear velocity (world frame) (init=0) */
    Vector2d v = new Vector2d(); 

    /** Orientation angle (init=0) */
    double   theta = 0;

    /** Angular velocity (init=0) */
    double   omega = 0;

    /** Accumulator for linear force (world frame) */
    Vector2d force  = new Vector2d(); 

    /** Accumulator for angular force, i.e., torque (world frame) */
    double   torque = 0;

    /** Pin-constraint status. */
    boolean pin = false;

    /** Block approximation of geometry for rendering and collision
     * processing (body frame). */
    ArrayList<Block> B = new ArrayList<Block>();

    /** Boundary blocks. */
    ArrayList<Block> boundaryBlocks = null;

    /** Disk that bounds blocks (body frame). */
    Disk boundingDisk;

    /** Index of body, e.g., for collision detection id. */
    private int key = -1;

    /** body2world transform */
    RigidTransform transformB2W = new RigidTransform();

    /** world2body transform */
    RigidTransform transformW2B = new RigidTransform();

    /** 
     * Constructs rigid body as the union of specified blocks.
     * @param blocks Blocks in question. 
     */
    RigidBody(Collection<Block> blocks) 
    {
	if(blocks.size() == 0) throw new IllegalArgumentException("RigidBody needs some input blocks");

	/// Avoid fragile reference to "blocks"
	B.addAll(blocks);

	/////////////////////////////////////////////////////////
	/// COMPUTE TOTAL MASS:
	massLinear = 0;
	for(Block b : B) {
	    massLinear += b.getColorMass();
	}

	/////////////////////////////////////////////////////////
	/// COMPUTE CENTER OF MASS:
	Vector2d xCM = new Vector2d();
	for(Block b : B) {
	    Utils.acc(xCM, b.getColorMass()/massLinear, b.p());
	}
	/// ... TRANSLATE BODY COORDINATE SYSTEM'S ORIGIN TO CENTER OF
	/// MASS BY MODIFYING BLOCK COORDINATES:
	for(Block b : B)  b.p.sub(xCM);
	/// RECORD xCM FOR INITIAL POSITION:
	x0.set(xCM);

	/////////////////////////////////////////////////////////
	/// COMPUTE INERTIA = sum_i mass_i radiusSq_i
	massAngular = 0;
	Vector2d r = new Vector2d();
	for(Block b : B)  {
	    r.set(b.p);
	    double radiusSquared = r.lengthSquared();
	    massAngular  +=  b.getColorMass() * (radiusSquared + b.h()*b.h()); /// +h*h to avoid zero inertia
	}
	if(massAngular==0) throw new RuntimeException("massAngular was 0;  |blocks|="+blocks.size());

	/// COMPUTE BOUNDING DISK (body frame):
	boundingDisk = new Disk(B);

	//System.out.println("RigidBody(): m="+massLinear+", I="+massAngular);

	/////////////////////////////////////////////////////////
	/// RESET STATE (pos/vel) AND CO-STATE (force/torque):
	reset();
	
	//System.out.println("RigidBody: |blocks|="+blocks.size());
    }

    /** Copy constructor that shares underlying Block objects. */
    RigidBody(RigidBody body) {
	this(body.B);
    }

    /** Integer key used to identify body in collision processing,
     * etc. (currently called by CollisionProcessor) */
    void setKey(int key) {
	this.key = key;
    }
    /** Integer key used to identify body in collision processing
     * (default=-1 if not set). */
    public int getKey() { return key; }


    /** Specifies whether or not this rigidbody is fixed in space via
     * a full-dof (encastre) pin constraint.  */
    public void setPin(boolean fix) { pin = fix; }

    /** Returns true if currently pinned. */
    public boolean isPinned() { return pin; }

    /** Returns true if all blocks are shades of pure blue, c \propto (0,0,1) or (a,a,b) with b>a. */
    boolean hasAllBlueBlocks()
    {
	for(Block b : B) {
	    boolean isBlue = ((b.c.x == b.c.y) && (b.c.z > b.c.x));
	    if(!isBlue) return false;
	}
	return true;
    }


    /** Returns true if point intersects support of body (HACK:
     * Currently just bounding Disk... you may want to use a hierarchy
     * here, especially to support high-resolution picking.).  */
    public boolean intersectsW(Point2d pointW)
    {
	Point2d pointB = new Point2d(pointW);//sloth
	transformW2B.transform(pointB);
	return boundingDisk.intersects(pointB);
    }

    /** Number of image blocks comprising this rigid body. */
    public int getNBlocks() { 
	return B.size();
    }

    /** Fragile reference to all blocks. */
    Collection<Block> getBlocks() {
	return B;
    }

    /** Fragile reference to blocks without 8 equidistant neighbour blocks. */
    Collection<Block> getBoundaryBlocks() 
    {
	if(boundaryBlocks == null) {///COMPUTE -- SLOW :(

	    if(B.size() > 100) System.out.print("RigidBody.getBoundaryBlocks(): ALL-PAIRS SEARCH: |boundaryBlocks| = ");

	    boundaryBlocks = new ArrayList<Block>();

	    for(Block b : B) {
		
		double H      = 2.*b.h();
		double R2     = (1.43*H)*(1.43*H);
		int    nNeigh = 0;
		for(Block c : B) {
		    if(c != b) {
			if(c.p.distanceSquared(b.p) < R2)  nNeigh++;
		    }
		}
		
		//System.out.print(" "+nNeigh+" ");
		if(nNeigh < 8) boundaryBlocks.add(b);
	    }
	    if(B.size() > 100) System.out.println(boundaryBlocks.size()+"  ;P");
	}
	return boundaryBlocks;
    }

    /** Returns true if bounds of the two bodies intersect (default is
     * Disk-Disk test, but an AABB-AABB may perform better for
     * text/images.). */
    public boolean intersectsBounds(RigidBody X) 
    {
	Point2d c = new Point2d();//tmp to map X.disk.c to this body frame
	c.set(X.boundingDisk.c);
	X.transformB2W(c);
	transformW2B(c);
	double sep = boundingDisk.r + X.boundingDisk.r;
	return (boundingDisk.c.distanceSquared(c) < sep*sep);
    }

    /** Bounding disk in body (B) coordinates. */
    public Disk getBoundingDiskB() { return boundingDisk; }

    /** Mass of object. */
    public double getMass() { return massLinear; }

    /** Angular mass, or inertia tensor I_zz, of object.*/
    public double getMassAngular() { return massAngular; }

    /** Both linear and angular kinetic energy. */
    public double getKineticEnergy() {
	return 0.5*massLinear*v.dot(v) + 0.5*massAngular*omega*omega;
    }

    /** Fragile reference to center-of-mass position. */
    Point2d getPosition() { return x; }

    /** Current rotation angle (in radians). */
    double getOrientation() { return theta; }

    /** Fragile reference to linear velocity. */
    Vector2d getVelocityLinear() { return v; }

    /** Fragile reference to linear velocity. */
    double getVelocityAngular() { return omega; }

    /** New Vector2d containing spatial velocity of the specified world-frame point. */
    Vector2d getSpatialVelocityW(Point2d contactPointW)
    {
	// ADD ROTATIONAL VEL:
	Vector2d pv = new Vector2d();
	pv.sub(contactPointW, x);// r = p - x
	pv.scale(omega);
	double vy = pv.y;
	pv.y = pv.x;
	pv.x = -vy;

	// ADD LINEAR VEL:
	pv.add(v);

	return pv;/// sloth "new"
    }


    /** Returns state info as a string. */
    public String toString() {
	return "RigidBody: x="+x+", theta="+theta+", v="+v+", omega="+omega;
    }

    /** 
     * TODO(ADD TORQUE SUPPORT!) Applies contact force (in world coordinates)
     * @param contactPointW Contact point in world coordinates
     * @param contactForceW Contact force in world coordinates
     */
    public void applyContactForceW(Point2d contactPointW, Vector2d contactForceW)
    {
	force.add(new Vector2d(contactForceW.x * 30, contactForceW.y*30));
	
	// ADD TORQUE:  TODO  ######
	Vector2d temp = new Vector2d();
	
	temp.sub(x, contactPointW);
//	torque += (contactForceW.x*temp.y) - (contactForceW.y*temp.x); 
    }

    /** Accumulates force/torque in world coordinates. Only affects
     * coming time step. */
    public void applyWrenchW(Vector2d f, double tau) 
    {
	//System.out.println("applyWrenchW: f="+(new Vector2f(f))+", tau="+tau);
	force.add(f);
	//torque += tau;
    }

    /** Resets state to original values (original
     * position/orientation, with zero velocities), updates rigid
     * transforms, and clears force/torque accumulators.  */
    public void reset() 
    { 
	/// RESET TO INITIAL STATE.
	x.set(x0);
	v.x = v.y = theta = omega = force.x = force.y = torque = 0;

	/// UPDATE transformB2W & transformW2B:
	updateRigidTransforms();
    }

    /** Refreshes transformB2W and transformW2B using current
     * position/orientation. */
    private void updateRigidTransforms()
    {
	transformB2W.set(theta, x);
	transformW2B.set(transformB2W);
	transformW2B.invert();
    }
    /** Transforms point/vector from World to Body frame. */
    public void transformW2B(Tuple2d x) {
	transformW2B.transform(x);
    }
    /** Transforms point/vector from Body to World frame. */
    public void transformB2W(Tuple2d x) {
	transformB2W.transform(x);
    }

    /** Advances body state, integrating any accumulated force/torque
     * (which are then set to zero), and updates internal rigid
     * transforms.   */
    public void advanceTime(double dt)
    {
	if(!pin) {/// NOT THE RIGHT INTEGRATOR! 

	    {/// SYMPLECTIC EULER
		/// UPDATE LINEAR POSITION/VELOCITY:
		Utils.acc(v,  dt/massLinear, force);// v' = v + dt*f/m
		Utils.acc(x, dt, v);

		/// TODO: UPDATE ANGULAR POSITION/VELOCITY:
		
	//  omega += dt*torque/massAngular;
	//	theta += dt*omega;
		
	    }

	    /// UPDATE RigidTransforms:
	    updateRigidTransforms();
	}

	/// RESET FORCE/TORQUE ACCUMULATORS:
	force.x = force.y = torque = 0;
    }


    private int displayList = -1;

    /** Draws body using a display list. */
    public synchronized void display(GL2 gl) 
    {
	/// SETUP TRANSFORM: 
	gl.glPushMatrix();
	{
	    applyGLTransform(gl);

	    {///DISPLAY:
		if(displayList < 0) {// MAKE DISPLAY LIST:
		    int displayListIndex = gl.glGenLists(1);
		    gl.glNewList(displayListIndex, GL2.GL_COMPILE);
		    drawBlocks(gl); 
		    gl.glEndList();
		    //System.out.println("MADE DISPLAY LIST "+displayListIndex+" : "+gl.glIsList(displayListIndex));
		    displayList = displayListIndex;
		}
		else {
		    gl.glCallList(displayList);
		}
	    }
	}
	gl.glPopMatrix();
    }

    /** Applies the body-to-world (B2W) transformation. */
    private void applyGLTransform(GL2 gl)
    {
	gl.glTranslated(x.x, x.y, 0);
	double angleInDegrees = 180./Math.PI * theta;
	gl.glRotated(angleInDegrees, 0, 0, 1);
    }

    private void drawBlocks(GL2 gl)
    {
	for(Block b : B) {
	    b.display(gl);
	}
    }

    /** Draws a Disk. */
    public void displayBound(GL2 gl)
    {
	gl.glPushMatrix();
	{
	    applyGLTransform(gl);

	    /// DRAW SINGLE-DISK BOUND:
// 	    gl.glColor3f(0.5f,0,0);
// 	    boundingDisk.display(gl);

	    /// DRAW BLOCK DISKS:
	    //double sqrt2 = 1.414213562373095;
	    double sqrt2 = 1.40;///slightly smaller for display purposes
	    gl.glColor3f(0.9f,.1f,.1f);
	    for(Block b : getBoundaryBlocks()) {
		Disk.displayDisk(gl, b.p(), sqrt2*b.h());
	    }	    
	}
	gl.glPopMatrix();
    }
}
