package cs5643.rigidbody;
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
}