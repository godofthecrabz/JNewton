package crab.newton;

import java.lang.foreign.*;

public class NewtonCollisionAggregate {
	
	protected final MemorySegment address;

	protected NewtonCollisionAggregate(MemorySegment address) {
		this.address = address;
	}
	
	public void destroy() {
		Newton.NewtonCollisionAggregateDestroy(address);
	}
	
	public void addBody(NewtonBody body) {
		Newton.NewtonCollisionAggregateAddBody(address, body.address);
	}
	
	public void removeBody(NewtonBody body) {
		Newton.NewtonCollisionAggregateRemoveBody(address, body.address);
	}
	
	public int getSelfCollision() {
		return Newton.NewtonCollisionAggregateGetSelfCollision(address);
	}
	
	public void setSelfCollision(int state) {
		Newton.NewtonCollisionAggregateSetSelfCollision(address, state);
	}
}
