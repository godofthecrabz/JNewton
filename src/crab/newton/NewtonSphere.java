package crab.newton;

import crab.newton.internal.Newton_h;
import java.lang.foreign.*;

public final class NewtonSphere implements NewtonCollision {
	
	private final MemoryAddress address;
	
	protected NewtonSphere(MemoryAddress address) {
		this.address = address;
	}
	
	/**
	 * 
	 * @param world
	 * @param radius
	 * @param shapeID
	 * @param offsetMatrix
	 * @param allocator
	 * @return
	 */
	public static NewtonSphere create(NewtonWorld world, float radius, int shapeID, float[] offsetMatrix, SegmentAllocator allocator) {
		if (offsetMatrix == null) {
			return new NewtonSphere(Newton_h.NewtonCreateSphere(world.address, radius, shapeID, MemoryAddress.NULL));
		}
		MemorySegment matrix = allocator.allocateArray(Newton_h.C_FLOAT, offsetMatrix);
		return new NewtonSphere(Newton_h.NewtonCreateSphere(world.address, radius, shapeID, matrix));
	}

	@Override
	public MemoryAddress address() {
		return address;
	}
}
