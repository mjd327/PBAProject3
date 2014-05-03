package cs5643.rigidbody;

import javax.vecmath.*;
import javax.media.opengl.*;

/** 
 * Spring force between one body and a proxy point. 
 * 
 * Requires modification to support mouse-spring forces between picked
 * non-center-of-mass positions on a rigid body.
 * 
 * @author Doug James, March 2007.
 */
public class SpringForcePoint2Body implements Force
{
    RigidBody       R;
    Point2d         x;
    RigidBodySystem S;

    SpringForcePoint2Body(RigidBody R, Point2d x, RigidBodySystem S)
    {
	if(R==null || x==null) throw new NullPointerException("R="+R+", x="+x);

	this.R = R;
	this.x = new Point2d(x);
	this.S = S;
    }

    public void updatePoint(Point2d xNew) {
	x.set(xNew);
    }

    public void applyForce()
    {
	/// HACK: APPLY SPRING FORCE TO CENTER OF MASS:
	/// TODO: SUPPORT FORCES ON PICKED POSITIONS
	Vector2d v = new Vector2d();
	v.sub(x, R.getPosition());
	double L = v.length();
	v.normalize();
	Vector2d vel = R.getVelocityLinear();//fragile ref
	//double vn = vel.dot(v);
	double k = Constants.STIFFNESS_STRETCH_DENSITY * R.getMass();
	v.scale( k*L ); //- 0.2*k*vn );//force
	R.applyWrenchW(v, 0);//at center of mass, so no torque

	/// DAMP RIGIDBODY MOTION
	v.set(vel);
	v.scale( -0.2 * k );
	R.applyWrenchW(v, 0);
    }

    public void display(GL2 gl)
    {
	/// DRAW A LINE:
	gl.glColor3f(0,1,0);
	gl.glBegin(GL2.GL_LINES);
	Point2d c = R.getPosition();
	gl.glVertex2d(c.x, c.y);
	gl.glVertex2d(x.x, x.y);
	gl.glEnd();	
    }

    public RigidBodySystem getSystem() { return S; }
}
