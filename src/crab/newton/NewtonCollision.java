package crab.newton;

import crab.newton.generated.Newton_h;

import jdk.incubator.foreign.*;

public interface NewtonCollision {
	
	MemoryAddress address();
	
	default int getMode() {
		return Newton_h.NewtonCollisionGetMode(address());
	}
	
	default void setMode(int mode) {
		Newton_h.NewtonCollisionSetMode(address(), mode);
	}
	
	default float calculateVolume() {
		return Newton_h.NewtonConvexCollisionCalculateVolume(address());
	}
	
	default float[] calculateInertiaMatrix() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment inertiaOrigin = allocator.allocateArray(Newton_h.C_FLOAT, Newton.AABBF);
			Newton_h.NewtonConvexCollisionCalculateInertialMatrix(address(), 
					inertiaOrigin.asSlice(0L, Newton_h.C_FLOAT.byteSize() * 3), 
					inertiaOrigin.asSlice(Newton_h.C_FLOAT.byteSize() * 3));
			return inertiaOrigin.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default int getType() {
		return Newton_h.NewtonCollisionGetType(address());
	}
	
	default boolean isConvex() {
		return Newton_h.NewtonCollisionIsConvexShape(address()) == 1 ? true : false;
	}
	
	default boolean isStatic() {
		return Newton_h.NewtonCollisionIsStaticShape(address()) == 1 ? true : false;
	}
	
	default void setUserData(Addressable data) {
		Newton_h.NewtonCollisionSetUserData(address(), data);
	}
	
	default MemoryAddress getUserData() {
		return Newton_h.NewtonCollisionGetUserData(address());
	}
	
	default void setUserID(long id) {
		Newton_h.NewtonCollisionSetUserID(address(), id);
	}
	
	default long getUserID() {
		return Newton_h.NewtonCollisionGetUserID(address());
	}
	
	default MemoryAddress getSubCollisionHandle() {
		return Newton_h.NewtonCollisionGetSubCollisionHandle(address());
	}
	
	default NewtonCollision getParentInstance() {
		MemoryAddress ptr = Newton_h.NewtonCollisionGetParentInstance(address());
		return ptr.equals(MemoryAddress.NULL) ? null : NewtonCollision.wrap(ptr);
	}
	
	default void setMatrix(float[] matrix) {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment matrixSegment = allocator.allocateArray(Newton_h.C_FLOAT, matrix);
			Newton_h.NewtonCollisionSetMatrix(address(), matrixSegment);
		}
	}
	
	default float[] getMatrix() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment matrixSegment = allocator.allocateArray(Newton_h.C_FLOAT, new float[16]);
			Newton_h.NewtonCollisionGetMatrix(address(), matrixSegment);
			return matrixSegment.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default void setScale(float x, float y, float z) {
		Newton_h.NewtonCollisionSetScale(address(), x, y, z);
	}
	
	default float[] getScale() {
		try (ResourceScope scope = ResourceScope.newConfinedScope()) {
			SegmentAllocator allocator = SegmentAllocator.nativeAllocator(scope);
			MemorySegment xyzSeg = allocator.allocateArray(Newton_h.C_FLOAT, new float[3]);
			Newton_h.NewtonCollisionGetScale(address(), 
					xyzSeg.asSlice(0L, Newton_h.C_FLOAT.byteSize()), 
					xyzSeg.asSlice(4L, Newton_h.C_FLOAT.byteSize()), 
					xyzSeg.asSlice(8L, Newton_h.C_FLOAT.byteSize()));
			return xyzSeg.toArray(Newton_h.C_FLOAT);
		}
	}
	
	default void destroy() {
		Newton_h.NewtonDestroyCollision(address());
	}
	
	default float getSkinThickness() {
		return Newton_h.NewtonCollisionGetSkinThickness(address());
	}
	
	default void setSkinThickness(float thickness) {
		Newton_h.NewtonCollisionSetSkinThickness(address(), thickness);
	}
	
	public static NewtonCollision wrap(MemoryAddress address) {
		int collisionType = Newton_h.NewtonCollisionGetType(address);
		return switch (collisionType) {
			case 0 -> new NewtonSphere(address);
			case 1 -> new NewtonCapsule(address);
			case 2 -> new NewtonCylinder(address);
			case 3 -> new NewtonChamferCylinder(address);
			case 4 -> new NewtonBox(address);
			case 5 -> new NewtonCone(address);
			case 6 -> new NewtonConvexHull(address);
			case 7 -> new NewtonNull(address);
			case 8 -> new NewtonCompoundCollision(address);
			case 9 -> new NewtonTreeCollision(address);
			case 10 -> new NewtonHeightField(address);
			default -> throw new RuntimeException("Error wrapping MemoryAddress");
		};
	}
}