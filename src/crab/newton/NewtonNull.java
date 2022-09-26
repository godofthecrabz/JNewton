package crab.newton;

import crab.newton.internal.Newton_h;
import java.lang.foreign.*;

public final class NewtonNull implements NewtonCollision {
	
	private final MemoryAddress address;
	
	protected NewtonNull(MemoryAddress address) {
		this.address = address;
	}
	
	/**
	 * 
	 * @param world
	 * @return
	 */
	public static NewtonNull create(NewtonWorld world) {
		return new NewtonNull(Newton_h.NewtonCreateNull(world.address));
	}

	@Override
	public MemoryAddress address() {
		return address;
	}
}
