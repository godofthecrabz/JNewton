package crab.newton;

import crab.newton.callbacks.NewtonCollisionTreeRayCastCallback;
import crab.newton.callbacks.NewtonHeightFieldRayCastCallback;
import crab.newton.callbacks.NewtonTreeCollisionFaceCallback;

import java.lang.foreign.*;

public final class NewtonCollision {

	public static final int SPHERE = 0,
			CAPSULE = 1,
			CYLINDER = 2,
			CHAMFERCYLINDER = 3,
			BOX = 4,
			CONE = 5,
			CONVEXHULL = 6,
			NULL = 7,
			COMPOUND = 8,
			TREE = 9,
			HEIGHTFIELD = 10,
			CLOTH_PATCH = 11,
			DEFORMABLE_SOLID = 12,
			USERMESH = 13,
			SCENE = 14,
			FRACTURED_COMPOUND = 15;
	protected final MemorySegment address;
	public final int collisionType;

	protected NewtonCollision(MemorySegment address) {
		this(address, Newton.NewtonCollisionGetType(address));
	}

	protected NewtonCollision(MemorySegment address, int collisionType) {
		this.address = address;
		this.collisionType = collisionType;
	}

	@Override
	public boolean equals(Object object) {
		return object instanceof NewtonCollision collision &&
				this.address.equals(collision.address) &&
				this.collisionType == collision.collisionType;
	}

	public NewtonMesh createMesh() {
		return new NewtonMesh(Newton.NewtonMeshCreateFromCollision(address));
	}
	
	public int getMode() {
		return Newton.NewtonCollisionGetMode(address);
	}

	public void setMode(int mode) {
		Newton.NewtonCollisionSetMode(address, mode);
	}

	public float calculateVolume() {
		return Newton.NewtonConvexCollisionCalculateVolume(address);
	}

	public void calculateInertiaMatrix(MemorySegment inertiaOrigin) {
		Newton.NewtonConvexCollisionCalculateInertialMatrix(address,
				inertiaOrigin.asSlice(0L, Newton.VEC3F.byteSize()),
				inertiaOrigin.asSlice(Newton.VEC3F.byteSize()));
	}

	public float[] calculateInertiaMatrix() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment inertiaOrigin = arena.allocate(Newton.AABBF);
			Newton.NewtonConvexCollisionCalculateInertialMatrix(address,
					inertiaOrigin.asSlice(0L, Newton.VEC3F.byteSize()),
					inertiaOrigin.asSlice(Newton.VEC3F.byteSize()));
			return inertiaOrigin.toArray(Newton.C_FLOAT);
		}
	}

	public boolean isConvex() {
		return Newton.NewtonCollisionIsConvexShape(address) == 1;
	}

	public boolean isStatic() {
		return Newton.NewtonCollisionIsStaticShape(address) == 1;
	}

	public void setUserData(MemorySegment data) {
		Newton.NewtonCollisionSetUserData(address, data);
	}

	public MemorySegment getUserData() {
		return Newton.NewtonCollisionGetUserData(address);
	}

	public void setUserID(long id) {
		Newton.NewtonCollisionSetUserID(address, id);
	}

	public long getUserID() {
		return Newton.NewtonCollisionGetUserID(address);
	}

	public MemorySegment getSubCollisionHandle() {
		return Newton.NewtonCollisionGetSubCollisionHandle(address);
	}

	public NewtonCollision getParentInstance() {
		MemorySegment ptr = Newton.NewtonCollisionGetParentInstance(address);
		return ptr.equals(MemorySegment.NULL) ? null : NewtonCollision.wrap(ptr);
	}

	public void setMatrix(MemorySegment matrix) {
		Newton.NewtonCollisionSetMatrix(address, matrix);
	}

	public void setMatrix(float[] matrix) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSegment = arena.allocateArray(Newton.C_FLOAT, matrix);
			Newton.NewtonCollisionSetMatrix(address, matrixSegment);
		}
	}

	public void getMatrix(MemorySegment matrix) {
		Newton.NewtonCollisionGetMatrix(address, matrix);
	}

	public float[] getMatrix() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSegment = arena.allocate(Newton.MAT4F);
			Newton.NewtonCollisionGetMatrix(address, matrixSegment);
			return matrixSegment.toArray(Newton.C_FLOAT);
		}
	}

	public void setScale(float x, float y, float z) {
		Newton.NewtonCollisionSetScale(address, x, y, z);
	}

	public void getScale(MemorySegment vector) {
		Newton.NewtonCollisionGetScale(address,
				vector.asSlice(0L, Newton.C_FLOAT.byteSize()),
				vector.asSlice(4L, Newton.C_FLOAT.byteSize()),
				vector.asSlice(8L, Newton.C_FLOAT.byteSize()));
	}

	public float[] getScale() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment xyzSeg = arena.allocate(Newton.VEC3F);
			Newton.NewtonCollisionGetScale(address,
					xyzSeg.asSlice(0L, Newton.C_FLOAT.byteSize()), 
					xyzSeg.asSlice(4L, Newton.C_FLOAT.byteSize()), 
					xyzSeg.asSlice(8L, Newton.C_FLOAT.byteSize()));
			return xyzSeg.toArray(Newton.C_FLOAT);
		}
	}

	public void destroy() {
		Newton.NewtonDestroyCollision(address);
	}

	public float getSkinThickness() {
		return Newton.NewtonCollisionGetSkinThickness(address);
	}

	public void setSkinThickness(float thickness) {
		Newton.NewtonCollisionSetSkinThickness(address, thickness);
	}

	//Compound & Scene Collision methods

	public void beginAddRemove() {
		switch (collisionType) {
			case COMPOUND -> Newton.NewtonCompoundCollisionBeginAddRemove(address);
			case SCENE -> Newton.NewtonSceneCollisionBeginAddRemove(address);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		}
	}

	public MemorySegment addSubCollision(NewtonCollision collision) {
		return switch (collisionType) {
			case COMPOUND -> Newton.NewtonCompoundCollisionAddSubCollision(address, collision.address);
			case SCENE -> Newton.NewtonSceneCollisionAddSubCollision(address, collision.address);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		};
	}

	public void removeSubCollision(MemorySegment collisionNode) {
		switch (collisionType) {
			case COMPOUND -> Newton.NewtonCompoundCollisionRemoveSubCollision(address, collisionNode);
			case SCENE -> Newton.NewtonSceneCollisionRemoveSubCollision(address, collisionNode);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		}
	}

	public void removeSubCollisionByIndex(int index) {
		switch (collisionType) {
			case COMPOUND -> Newton.NewtonCompoundCollisionRemoveSubCollisionByIndex(address, index);
			case SCENE -> Newton.NewtonSceneCollisionRemoveSubCollisionByIndex(address, index);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		}
	}

	public void setSubCollisionMatrix(MemorySegment collisionNode, float[] matrix) {
		//switch (collisionType) {
		//	case COMPOUND -> Newton.NewtonCompoundCollisionRemoveSubCollisionByIndex(address, index);
		//	case SCENE -> Newton.NewtonSceneCollisionRemoveSubCollisionByIndex(address, index);
		//	default -> throw new RuntimeException("Collision Type incompatible with this method");
		//}
	}

	public void endAddRemove() {
		switch (collisionType) {
			case COMPOUND -> Newton.NewtonCompoundCollisionEndAddRemove(address);
			case SCENE -> Newton.NewtonSceneCollisionEndAddRemove(address);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		}
	}

	public MemorySegment getFirstNode() {
		return switch (collisionType) {
			case COMPOUND -> Newton.NewtonCompoundCollisionGetFirstNode(address);
			case SCENE -> Newton.NewtonSceneCollisionGetFirstNode(address);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		};
	}

	public MemorySegment getNextNode(MemorySegment nextNode) {
		return switch (collisionType) {
			case COMPOUND -> Newton.NewtonCompoundCollisionGetNextNode(address, nextNode);
			case SCENE -> Newton.NewtonSceneCollisionGetNextNode(address, nextNode);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		};
	}

	public MemorySegment getNodeByIndex(int index) {
		return switch (collisionType) {
			case COMPOUND -> Newton.NewtonCompoundCollisionGetNodeByIndex(address, index);
			case SCENE -> Newton.NewtonSceneCollisionGetNodeByIndex(address, index);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		};
	}

	public int getNodeIndex(MemorySegment collisionNode) {
		return switch (collisionType) {
			case COMPOUND -> Newton.NewtonCompoundCollisionGetNodeIndex(address, collisionNode);
			case SCENE -> Newton.NewtonSceneCollisionGetNodeIndex(address, collisionNode);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		};
	}

	public NewtonCollision getCollisionFromNode(MemorySegment collisionNode) {
		return switch (collisionType) {
			case COMPOUND -> NewtonCollision.wrap(Newton.NewtonCompoundCollisionGetCollisionFromNode(address, collisionNode));
			case SCENE -> NewtonCollision.wrap(Newton.NewtonSceneCollisionGetCollisionFromNode(address, collisionNode));
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		};
	}

	//Convex Hull collision methods

	public int getFaceIndices(int face, int[] faceIndices) {
		return 0;
	}

	//HeightField/Tree collision methods

	public void setUserRaycastCallback(MemorySegment callback) {
		switch (collisionType) {
			case HEIGHTFIELD -> Newton.NewtonHeightFieldSetUserRayCastCallback(address, callback);
			case TREE -> Newton.NewtonTreeCollisionSetUserRayCastCallback(address, callback);
			default -> throw new RuntimeException("Collision Type incompatible with this method");
		}
	}

	public void setUserRaycastCallback(NewtonHeightFieldRayCastCallback rayHitCallback, SegmentScope scope) {
		if (collisionType != HEIGHTFIELD) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		MemorySegment rayHitCallbackFunc = NewtonHeightFieldRayCastCallback.allocate(rayHitCallback, scope);
		Newton.NewtonHeightFieldSetUserRayCastCallback(address, rayHitCallbackFunc);
	}

	public void setUserRayCastCallback(NewtonCollisionTreeRayCastCallback rayHitCallback, SegmentScope scope) {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		MemorySegment rayHitCallbackFunc = NewtonCollisionTreeRayCastCallback.allocate(rayHitCallback, scope);
		Newton.NewtonTreeCollisionSetUserRayCastCallback(address, rayHitCallbackFunc);
	}

	public void beginBuild() {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		Newton.NewtonTreeCollisionBeginBuild(address);
	}

	public void addFace(int vertexCount, MemorySegment vertexList, int strideInBytes, int faceAttribute) {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		Newton.NewtonTreeCollisionAddFace(address, vertexCount, vertexList, strideInBytes, faceAttribute);
	}

	public void addFace(int vertexCount, float[] vertexList, int strideInBytes, int faceAttribute) {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		try (Arena arena = Arena.openConfined()) {
			MemorySegment vertexSegment = arena.allocateArray(Newton.C_FLOAT, vertexList);
			Newton.NewtonTreeCollisionAddFace(address, vertexCount, vertexSegment, strideInBytes, faceAttribute);
		}
	}

	public void endBuild(int optimize) {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		Newton.NewtonTreeCollisionEndBuild(address, optimize);
	}

	public int getFaceAttribute(MemorySegment faceIndexArray, int indexCount) {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		return Newton.NewtonTreeCollisionGetFaceAttribute(address, faceIndexArray, indexCount);
	}

	public int getFaceAttribute(int[] faceIndexArray, int indexCount) {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		try (Arena arena = Arena.openConfined()) {
			MemorySegment indexSegment = arena.allocateArray(Newton.C_INT, faceIndexArray);
			return Newton.NewtonTreeCollisionGetFaceAttribute(address, indexSegment, indexCount);
		}
	}

	public void setFaceAttribute(MemorySegment faceIndexArray, int indexCount, int attribute) {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		Newton.NewtonTreeCollisionSetFaceAttribute(address, faceIndexArray, indexCount, attribute);
	}

	public void setFaceAttribute(int[] faceIndexArray, int indexCount, int attribute) {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		try (Arena arena = Arena.openConfined()) {
			MemorySegment indexSegment = arena.allocateArray(Newton.C_INT, faceIndexArray);
			Newton.NewtonTreeCollisionSetFaceAttribute(address, indexSegment, indexCount, attribute);
		}
	}

	public void forEachFace(MemorySegment callback, MemorySegment context) {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		Newton.NewtonTreeCollisionForEachFace(address, callback, context);
	}

	public void forEachFace(NewtonTreeCollisionFaceCallback forEachFaceCallback, MemorySegment context, SegmentScope scope) {
		if (collisionType != TREE) {
			throw new RuntimeException("Collision Type incompatible with this method");
		}
		MemorySegment forEachFunc = NewtonTreeCollisionFaceCallback.allocate(forEachFaceCallback, scope);
		Newton.NewtonTreeCollisionForEachFace(address, forEachFunc, context);
	}
	
	public static NewtonCollision wrap(MemorySegment address) {
		int collisionType = Newton.NewtonCollisionGetType(address);
		return new NewtonCollision(address, collisionType);
	}
}