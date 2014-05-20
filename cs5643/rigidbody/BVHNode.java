package cs5643.rigidbody;
import java.awt.Color;

import javax.media.opengl.GL2;
import javax.vecmath.*;

/**
 * A class representing a node in a bounding volume hierarchy.
 */

public class BVHNode {
	
	public final Point2d minBound, maxBound;

	public final BVHNode child[];

	/** The index of the first block contained in this node */
	public int blockIndexStart;

	/** The index of the surface next to the last block under this node. */
	public int blockIndexEnd; 

	/** Constructor */
	public BVHNode(Point2d minBound, Point2d maxBound, BVHNode leftChild, BVHNode rightChild, int start, int end) 
	{
		this.minBound = new Point2d();
		this.minBound.set(minBound);
		this.maxBound = new Point2d();
		this.maxBound.set(maxBound);
		this.child = new BVHNode[2];
		this.child[0] = leftChild;
		this.child[1] = rightChild;		   
		this.blockIndexStart = start;
		this.blockIndexEnd = end;
	}

	public boolean isLeaf()
	{
		return child[0] == null && child[1] == null; 
	}

	public boolean intersects(RigidBody body)
	{
		return body.getMinBound().x <= maxBound.x && body.getMaxBound().x >= minBound.x
			&& body.getMinBound().y <= maxBound.y && body.getMaxBound().y >= minBound.y;
	}
	
	public void draw(GL2 gl)
	{
		if (isLeaf())
		{
			Color3f fillColor = new Color3f(Color.RED);
			gl.glColor4f(fillColor.x,fillColor.y,fillColor.z,.8f);
			gl.glBegin(GL2.GL_QUADS);
			gl.glVertex2d(minBound.x,minBound.y);
			gl.glVertex2d(maxBound.x,minBound.y);
			gl.glVertex2d(maxBound.x,maxBound.y);
			gl.glVertex2d(minBound.x,maxBound.y);
			gl.glEnd();
		}
		
		if (child[0] != null)
			child[0].draw(gl);
		if (child[1] != null)
			child[1].draw(gl);
	}

}