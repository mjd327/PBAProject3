package cs5643.rigidbody;
import javax.vecmath.*;


/**Class for axis aligned bounding boxes to speed up rigid body collision detection and handling
 * Code adapted from CS4620 Ray Tracing project */
public class BVH 
{
	/** A shared blocks array that will be used across every node in the tree */
	Block[] blocks;
	
	/** The root of the BVH tree */
	BVHNode root;
	
	/** Intersects the unpinned body with the boundary objects in the scene */
	public boolean intersect(RigidBody unpinnedBody, IntersectionRecord ir)
	{
		return intersectHelper(root, unpinnedBody, ir);
	}
	
	private boolean intersectHelper(BVHNode node, RigidBody unpinnedBody, IntersectionRecord ir)
	{
		// For a leaf node, use a normal linear search. Otherwise, search in the left and right children.
		// Save time by checking if the ray intersects the node first before checking the children.
		Point2d temp = new Point2d();
		if(node.intersects(unpinnedBody))
		{
			boolean intersected = false; 
			if(node.isLeaf())
			{
				for(Block b: blocks)
				{
					temp.set(b.p());
					b.body.transformB2W(temp);//-->world
					if (unpinnedBody.intersectsW(temp))
						ir.candidateBlocks.add(b);
				}
			}
			else
			{
				if(intersectHelper(node.child[0], unpinnedBody, ir))
				{
					intersected = true; 
				}
				
				if (intersectHelper(node.child[1], unpinnedBody, ir))
				{
					intersected = true; 
				}
			}

			return intersected;
		}
		return false;
	}
	
	private BVHNode createTree(int start, int end)
	{
		//Helpful variables we will use for our calculations
		Point2d minB;   //min bound of box enclosing all blocks
		Point2d maxB;   //max bound of box enclosing all blocks
		int widestDim;  //Widest dimension of box enclosing all blocks
		Point2d aveAveragePosition = new Point2d(0,0); //The average of the average positions
		
		Point2d tempLoc;
		
		// ==== Step 1 ====
		// Find out the BIG bounding box enclosing all the bodies in the range [start, end)
		// and store them in minB and maxB.
		minB = new Point2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
		maxB = new Point2d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

		//Go through all blocks in range, and find min and max values of x,y,and z
		for (int i = start;  i < end; i++)
		{
			tempLoc = new Point2d(blocks[i].p());
			blocks[i].body.transformB2W(tempLoc);//-->world
			if(tempLoc.x < minB.x) minB.x = tempLoc.x; 
			if(tempLoc.y < minB.y) minB.y = tempLoc.y; 
			
			if(tempLoc.x > maxB.x) maxB.x = tempLoc.x; 
			if(tempLoc.x > maxB.y) maxB.y = tempLoc.y; 
			
			//Compute sum of average positions to be used later
			aveAveragePosition.add(tempLoc);	
		}
		//Compute average of average positions.
		aveAveragePosition.scale(1.0/(end - start)); 
		
		// ==== Step 2 ====
		// Check for the base case. 
		// If the range [start, end) is small enough, just return a new leaf node.
		if(end - start <= 5) 
		{
			return new BVHNode(minB,maxB,null,null,start,end);
		}
		
		// ==== Step 3 ====
		// Figure out the widest dimension (x or y or z).
		// If x is the widest, set widestDim = 0. If y, set widestDim = 1. If z, set widestDim = 2.
		double diffx = maxB.x - minB.x;
		double diffy = maxB.y - minB.y;
		double maxDiff = Math.max(diffx,diffy); 
		if (maxDiff == diffx) widestDim = 0;
		else widestDim = 1;
		

		// ==== Step 4 ====
		// Sort bodies according to the widest dimension.
		// You can also implement O(n) randomized splitting algorithm.
		
		//We need to know the average along the widest dimension.
		double widestDimAverage = (widestDim == 0 ? aveAveragePosition.x : aveAveragePosition.y);
		
		//We will sort these by average along the widest dimension using a temporary array.
		Block[] tempBlocks = new Block[end-start]; 
		int leftCounter = start;
		int rightCounter = end-1; 
		double currentAverage; 
		Block curBlock;
		Point2d curBlockPos = new Point2d();
		boolean toLeft = true; 
		for (int i = start;  i < end; i++)
		{
			curBlock = blocks[i];
			curBlockPos.set(blocks[i].p());
			blocks[i].body.transformB2W(curBlockPos);//-->world
			
			//Find which dimension we need to sort by averages
			currentAverage = widestDim == 0 ? curBlockPos.x : curBlockPos.y;
			
			//We will insert lower values from the left, and higher from the right. 
			if(currentAverage < widestDimAverage || (currentAverage == widestDimAverage && toLeft))
			{
				tempBlocks[leftCounter - start] =  curBlock; 
				leftCounter++;
			}
			else if (currentAverage > widestDimAverage || (currentAverage == widestDimAverage && !toLeft))
			{
				tempBlocks[rightCounter - start] = curBlock;
				rightCounter--;
			}
			if (currentAverage == widestDimAverage)
			{
				toLeft = !toLeft; 
			}
		}
		//When we exit our for loop above, our temporary surfaces array contains all values sorted into two 
		//groups, with leftCounter being at the start of the second group, and right Counter the end of the first.
		//We must now replace the values in surfaces. 
		for(int i = start; i < end; i++)
		{
			blocks[i] = tempBlocks[i - start];
		}

		
		// ==== Step 5 ====
		// Recursively create left and right children.
		// As stated above, left counter is at start of right child, right counter at end of left child
		
		//System.out.println("Left Child: [" + start + ", " + leftCounter + ")");
		//System.out.println("Right Child: [" + leftCounter + ", " + end + ")");
		
		return new BVHNode(minB,maxB,createTree(start,leftCounter),createTree(leftCounter,end),start,end);				
	}	
	public void build(Block[] blocks)
	{
		this.blocks = blocks;
		System.out.println("Building tree again");
		root = createTree(0, blocks.length);
	}
}
