package crab.newton;

import crab.newton.generated.*;

import jdk.incubator.foreign.*;

public final class NewtonBox implements NewtonCollision {
	
	private final MemoryAddress address;
	
	/**
	 * Internal NewtonBox Constructor
	 * @param address
	 */
	protected NewtonBox(MemoryAddress address) {
		this.address = address;
	}
	
	/**
	 * Creates an instance of a box collision
	 * @param world - NewtonWorld
	 * @param dx
	 * @param dy
	 * @param dz
	 * @param shapeID
	 * @param offsetMatrix
	 * @param allocator
	 * @return
	 */
	public static NewtonBox create(NewtonWorld world, float dx, float dy, float dz, int shapeID, float[] offsetMatrix, SegmentAllocator allocator) {
		if (offsetMatrix == null) {
			return new NewtonBox(Newton_h.NewtonCreateBox(world.address, dx, dy, dz, shapeID, MemoryAddress.NULL));
		}
		MemorySegment matrix = allocator.allocateArray(Newton_h.C_FLOAT, offsetMatrix);
		return new NewtonBox(Newton_h.NewtonCreateBox(world.address, dx, dy, dz, shapeID, matrix));
	}

	@Override
	public MemoryAddress address() {
		return address;
	}
}
