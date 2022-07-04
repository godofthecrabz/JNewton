package crab.newton;

import java.nio.charset.StandardCharsets;
import java.util.Objects;

import crab.newton.generated.*;

import jdk.incubator.foreign.*;

public class NewtonHeightField implements NewtonCollision {
	
	private final MemoryAddress address;
	
	protected NewtonHeightField(MemoryAddress address) {
		this.address = address;
	}
	
	/**
	 * 
	 * @param world
	 * @param width
	 * @param height
	 * @param gridsDiagonals
	 * @param elevationdatType
	 * @param elevationMap
	 * @param attributeMap
	 * @param verticalScale
	 * @param horizontalScale_x
	 * @param horizontalScale_z
	 * @param shapeID
	 * @param allocator
	 * @return
	 */
	public static NewtonHeightField create(NewtonWorld world, int width,  int height,  int gridsDiagonals,  int elevationdatType,  Object elevationMap,  char[] attributeMap,  float verticalScale,  float horizontalScale_x,  float horizontalScale_z,  int shapeID, SegmentAllocator allocator) {
		Objects.requireNonNull(elevationMap);
		MemorySegment elevationSeg;
		if (!elevationMap.getClass().isArray()) {
			throw new IllegalArgumentException();
		} else {
			elevationSeg = switch (elevationdatType) {
				case 0 -> allocator.allocateArray(Newton_h.C_FLOAT, (float[]) elevationMap);
				case 1 -> allocator.allocateArray(Newton_h.C_SHORT, (short[]) elevationMap);
				default -> throw new IllegalArgumentException();
			};
		}
		MemorySegment attributeSeg = allocator.allocateArray(Newton_h.C_CHAR, new String(attributeMap).getBytes(StandardCharsets.UTF_8));
		return new NewtonHeightField(Newton_h.NewtonCreateHeightFieldCollision(world.address, width, height, gridsDiagonals, elevationdatType, elevationSeg, attributeSeg, verticalScale, horizontalScale_x, horizontalScale_z, shapeID));
	}
	
	public void setUserRaycastCallback(NewtonHeightFieldRayCastCallback rayHitCallback, ResourceScope scope) {
		NativeSymbol rayHitCallbackFunc = NewtonHeightFieldRayCastCallback.allocate(rayHitCallback, scope);
		Newton_h.NewtonHeightFieldSetUserRayCastCallback(address, rayHitCallbackFunc);
	}

	@Override
	public MemoryAddress address() {
		return address;
	}
}
