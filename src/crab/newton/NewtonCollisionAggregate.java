package crab.newton;

import crab.newton.internal.Newton_h;
import java.lang.foreign.MemoryAddress;

public class NewtonCollisionAggregate {
	
	protected final MemoryAddress address;

	protected NewtonCollisionAggregate(MemoryAddress address) {
		this.address = address;
	}
	
	public static NewtonCollisionAggregate create(NewtonWorld world) {
		return new NewtonCollisionAggregate(Newton_h.NewtonCollisionAggregateCreate(world.address));
	}
	
	public void destroy() {
		Newton_h.NewtonCollisionAggregateDestroy(address);
	}
	
	public void addBody(NewtonBody body) {
		Newton_h.NewtonCollisionAggregateAddBody(address, body.address());
	}
	
	public void removeBody(NewtonBody body) {
		Newton_h.NewtonCollisionAggregateRemoveBody(address, body.address());
	}
	
	public int getSelfCollision() {
		return Newton_h.NewtonCollisionAggregateGetSelfCollision(address);
	}
	
	public void setSelfCollision(int state) {
		Newton_h.NewtonCollisionAggregateSetSelfCollision(address, state);
	}
}
