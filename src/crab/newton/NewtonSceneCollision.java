package crab.newton;

import crab.newton.generated.Newton_h;

import jdk.incubator.foreign.*;

public class NewtonSceneCollision implements NewtonCollision {
	
	private final MemoryAddress address;
	
	protected NewtonSceneCollision(MemoryAddress address) {
		this.address = address;
	}
	
	/**
	 * 
	 * @param world
	 * @param shapeID
	 * @return
	 */
	public static NewtonSceneCollision create(NewtonWorld world, int shapeID) {
		return new NewtonSceneCollision(Newton_h.NewtonCreateSceneCollision(world.address, shapeID));
	}
	
	public void beginAddRemove() {
		Newton_h.NewtonSceneCollisionBeginAddRemove(address);
	}
	
	public Node addSubCollision(NewtonCollision collision) {
		return new Node(Newton_h.NewtonSceneCollisionAddSubCollision(address, collision.address()));
	}
	
	public void removeSubCollision(Node collisionNode) {
		Newton_h.NewtonSceneCollisionRemoveSubCollision(address, collisionNode.address());
	}
	
	public void removeSubCollisionByIndex(int index) {
		Newton_h.NewtonSceneCollisionRemoveSubCollisionByIndex(address, index);
	}
	
	public void setSubCollisionMatrix(Node collisionNode, float[] matrix) {
		//NewtonSceneCollisionSetSubCollisionMatrix
	}
	
	public void endAddRemove() {
		Newton_h.NewtonSceneCollisionEndAddRemove(address);
	}
	
	public Node getFirstNode() {
		return new Node(Newton_h.NewtonSceneCollisionGetFirstNode(address));
	}
	
	public Node getNextNode(Node nextNode) {
		MemoryAddress nodePtr = Newton_h.NewtonSceneCollisionGetNextNode(address, nextNode.address());
		return nodePtr.equals(MemoryAddress.NULL) ? null : new Node(nodePtr);
	}
	
	public Node getNodeByIndex(int index) {
		return new Node(Newton_h.NewtonSceneCollisionGetNodeByIndex(address, index));
	}
	
	public int getNodeIndex(Node collisionNode) {
		return Newton_h.NewtonSceneCollisionGetNodeIndex(address, collisionNode.address());
	}
	
	public NewtonCollision getCollisionFromNode(Node collisionNode) {
		return NewtonCollision.wrap(Newton_h.NewtonSceneCollisionGetCollisionFromNode(address, collisionNode.address()));
	}
	
	public record Node(MemoryAddress address) {}

	@Override
	public MemoryAddress address() {
		return address;
	}
}
