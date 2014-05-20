package cs5643.rigidbody;

import java.util.ArrayList;

public class IntersectionRecord
{
	ArrayList<Disk> candidateDisks= new ArrayList<Disk>();
	ArrayList<RigidBody> candidateBodies = new ArrayList<RigidBody>();
	ArrayList<Block> candidateBlocks = new ArrayList<Block>();
}
