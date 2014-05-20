package cs5643.rigidbody;

import java.util.*;

import javax.vecmath.*;

/**
 * Class to detect and resolve collisions (e.g., using penalty forces)
 * between rigid bodies. Implement both broad and narrow phase checks
 * here.
 * 
 * @author Doug James, March 2007.
 */
public class CollisionProcessor
{
	private RigidBody unpinnedBody;
	private RigidBody[] B;
	private Block[] blocks;
	private ArrayList<Block> candidateBlocks = new ArrayList<Block>();
	private HashSet<BodyPair> candidateBodyPairs = new HashSet<BodyPair>();
	private BVH bvh;

	/**
	 * Index bodies, and BUILD YOUR FAVORITE DATA STRUCTURES!
	 */
	CollisionProcessor(Set<RigidBody> bodies, BVH bvh)
	{
		B = (RigidBody[])bodies.toArray(new RigidBody[0]);

		/// INDEX BODIES (could use hash to avoid storing info in RigidBody):
		for(int k=0; k<B.length; k++)
		{
			B[k].setKey(k);
			if (!B[k].isPinned())
			{
				unpinnedBody = B[k];
			}		
		}
		this.bvh = bvh;
	}

	/**
	 * Performs broad and narrow phase collision detection, and
	 * applies penalty forces at contacts.
	 * @return 
	 */
	public boolean processCollisionsOLD()
	{
		/// BROAD PHASE:
		broadPhase();// --> updates candidateBodyPairs

		/// NARROW PHASE: Detect collisions and apply forces
		return narrowPhase();
	}
	
	public boolean processCollisions()
	{
		broadPhaseAABB();
		return narrowPhaseAABB();
	}

	/** Insert your implementation here of whatever broad phase test
	 * you choose. */
	void broadPhase()
	{
		/// TEMPORARY: Just do all Disk-Disk pairs (ha ha ha!)
		candidateBodyPairs.clear();
		for(int i=0; i<B.length; i++) 
			for(int j=0; j<i; j++) 
				if(B[i].intersectsBounds(B[j])) ///DISK TEST (NOT VERY ACCURATE), 
					candidateBodyPairs.add(new BodyPair(i,j));
				
		//System.out.println("BROAD PHASE: |candidateBodyPairs|="+candidateBodyPairs.size());
	}

	/** Insert your implementation of narrow phase collision detection
	 * and penalty force response here. Returns true if collision occurs anywhere.*/
	public boolean narrowPhase()
	{
		boolean collided = false; 
		/// TEMPORARY: All pairs... (ha ha ha)
		for(BodyPair pair : candidateBodyPairs) 
			collided = collided || processBodyPair(pair);
		return collided;
	}

	/** Narrow phase BodyPair collision resolution. You'll want to
	 * borrow ideas from this function for your narrow phase code.  */
	public boolean processBodyPair(BodyPair pair)
	{
		boolean collided = false; 
		int i = pair.i();
		int j = pair.j();
		if(i==j) return false;

		/// USE INTELLIGENT ALL-PAIRS TEST: 
		/// 
		/// TEMP/HACK... just use all pairs (ha ha ha):
		RigidBody bodyI = B[i];
		RigidBody bodyJ = B[j];
		double    massEff = 1./(1./bodyI.getMass() + 1./bodyJ.getMass());
		// 	Collection<Block> blocksI = bodyI.getBlocks();/// SLOTH: ONLY BOUNDARY BLOCKS NEEDED
		// 	Collection<Block> blocksJ = bodyJ.getBlocks();/// SLOTH: ONLY BOUNDARY BLOCKS NEEDED
		Collection<Block> blocksI = bodyI.getBoundaryBlocks();
		Collection<Block> blocksJ = bodyJ.getBoundaryBlocks();
		Point2d  piW = new Point2d();
		Point2d  pjW = new Point2d();
		Vector2d v   = new Vector2d();
		for(Block bi : blocksI)
		{
			piW.set(bi.p());//body coords
			bodyI.transformB2W(piW);//-->world

			for(Block bj : blocksJ) {

				/// SLOTH (REPEATED ~O(MN) times)
				pjW.set(bj.p());//body coords
				bodyJ.transformB2W(pjW);//-->world

				///TEST bi-bj AND APPLY WRENCH IF OVERLAPPING:
				v.sub(pjW,piW);
				double dist     = v.length() + 1.e-10;		
				double sumRadii = 1.41*(bi.h() + bj.h());
				double penDepth = sumRadii - dist;

				/// APPLY PENALTY FORCE IF IN CONTACT:
				if(penDepth > 0) {//overlap
					collided = true; 
					v.normalize();

					/// PENALTY CONTACT FORCE:
					double k     = Constants.CONTACT_STIFFNESS * massEff;
					double force =  k * penDepth;

					/// DAMPING: 
					double vi = v.dot(bodyI.getSpatialVelocityW(piW));
					double vj = v.dot(bodyJ.getSpatialVelocityW(pjW));
					double fDamp = + 0.004 * k * (vi - vj);

					v.scale(force + fDamp);
					bodyJ.applyContactForceW(pjW, v);
					v.negate();
					bodyI.applyContactForceW(piW, v);
				}
			}
		}
		return collided;
	}
	void broadPhaseAABB()
	{
		IntersectionRecord ir = new IntersectionRecord();
		candidateBlocks.clear();
		
		bvh.intersect(unpinnedBody, ir);
		
		for(Block cb: ir.candidateBlocks)
		{
			candidateBlocks.add(cb);
		}				
	}
	boolean narrowPhaseAABB()
	{
		boolean collided = false;
		
		for(Block b : candidateBlocks)
			collided = collided || processBlock(b);
		
		return collided;
	}
	public boolean processBlock(Block b)
	{
		boolean collided = false; 
		if(unpinnedBody.getBoundaryBlocks().contains(b))
			return false;

		double    massEff = 1./(1./unpinnedBody.getMass() + 1./b.body.getMass());
		Collection<Block> blocksUnpinned = unpinnedBody.getBoundaryBlocks();
		Point2d  piW = new Point2d();
		Point2d  pjW = new Point2d();
		Vector2d v   = new Vector2d();
		for(Block bi : blocksUnpinned) {

			piW.set(bi.p());//body coords
			unpinnedBody.transformB2W(piW);//-->world

			pjW.set(b.p());//body coords
			b.body.transformB2W(pjW);//-->world

			///TEST bi-bj AND APPLY WRENCH IF OVERLAPPING:
			v.sub(pjW,piW);
			double dist     = v.length() + 1.e-10;		
			double sumRadii = 1.41*(bi.h() + b.h());
			double penDepth = sumRadii - dist;

			/// APPLY PENALTY FORCE IF IN CONTACT:
			if(penDepth > 0) {//overlap
				collided = true; 
				v.normalize();

				/// PENALTY CONTACT FORCE:
				double k     = Constants.CONTACT_STIFFNESS * massEff;
				double force =  k * penDepth;

				/// DAMPING: 
				double vi = v.dot(unpinnedBody.getSpatialVelocityW(piW));
				double vj = v.dot(b.body.getSpatialVelocityW(pjW));
				double fDamp = + 0.004 * k * (vi - vj);

				v.scale(force + fDamp);
				b.body.applyContactForceW(pjW, v);
				v.negate();
				unpinnedBody.applyContactForceW(piW, v);

			}
		}
		return collided;
	}
	
}
