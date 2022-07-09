package crab.newton;

import crab.newton.generated.Newton_h;

import jdk.incubator.foreign.*;

public final class NewtonCompoundCollision implements NewtonCollision {
	
	private final MemoryAddress address;
	
	protected NewtonCompoundCollision(MemoryAddress address) {
		this.address = address;
	}
	
	public static NewtonCompoundCollision create(NewtonWorld world, int shapeID) {
		return new NewtonCompoundCollision(Newton_h.NewtonCreateCompoundCollision(world.address, shapeID));
	}
	
	public static NewtonCompoundCollision create(NewtonWorld world, NewtonMesh mesh, float hullTolerance, int shapeID, int subShapeID) {
		return new NewtonCompoundCollision(Newton_h.NewtonCreateCompoundCollisionFromMesh(world.address, mesh.address, hullTolerance, shapeID, subShapeID));
	}
	
	public void beginAddRemove() {
		Newton_h.NewtonCompoundCollisionBeginAddRemove(address);
	}
	
	public MemoryAddress addSubCollision(NewtonCollision convexCollision) {
		return Newton_h.NewtonCompoundCollisionAddSubCollision(address, convexCollision.address());
	}
	
	public void removeSubCollision(MemoryAddress collisionNode) {
		Newton_h.NewtonCompoundCollisionRemoveSubCollision(address, collisionNode);
	}
	
	public void removeSubCollisionByIndex(int index) {
		Newton_h.NewtonCompoundCollisionRemoveSubCollisionByIndex(address, index);
	}
	
	public void setSubCollisionMatrix(MemoryAddress collisionNode, float[] matrix) {
		//NewtonCompoundCollisionSetSubCollisionMatrix
	}
	
	public void endAddRemove() {
		Newton_h.NewtonCompoundCollisionEndAddRemove(address);
	}
	
	public MemoryAddress getFirstNode() {
		return Newton_h.NewtonCompoundCollisionGetFirstNode(address);
	}
	
	public MemoryAddress getNextNode(MemoryAddress nextNode) {
		return Newton_h.NewtonCompoundCollisionGetNextNode(address, nextNode);
	}
	
	public MemoryAddress getNodeByIndex(int index) {
		return Newton_h.NewtonCompoundCollisionGetNodeByIndex(address, index);
	}
	
	public int getNodeIndex(MemoryAddress collisionNode) {
		return Newton_h.NewtonCompoundCollisionGetNodeIndex(address, collisionNode);
	}
	
	public NewtonCollision getCollisionFromNode(MemoryAddress collisionNode) {
		return NewtonCollision.wrap(Newton_h.NewtonCompoundCollisionGetCollisionFromNode(address, collisionNode));
	}

	@Override
	public MemoryAddress address() {
		return address;
	}
}
