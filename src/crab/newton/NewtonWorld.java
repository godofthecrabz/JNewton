package crab.newton;

import crab.newton.callbacks.NewtonAllocMemory;
import crab.newton.callbacks.NewtonBodyIterator;
import crab.newton.callbacks.NewtonCollisionCopyConstructionCallback;
import crab.newton.callbacks.NewtonCollisionDestructorCallback;
import crab.newton.callbacks.NewtonContactsProcess;
import crab.newton.callbacks.NewtonCreateContactCallback;
import crab.newton.callbacks.NewtonDeserializeCallback;
import crab.newton.callbacks.NewtonDestroyContactCallback;
import crab.newton.callbacks.NewtonFreeMemory;
import crab.newton.callbacks.NewtonIslandUpdate;
import crab.newton.callbacks.NewtonJobTask;
import crab.newton.callbacks.NewtonJointIterator;
import crab.newton.callbacks.NewtonOnAABBOverlap;
import crab.newton.callbacks.NewtonOnBodyDeserializationCallback;
import crab.newton.callbacks.NewtonOnBodySerializationCallback;
import crab.newton.callbacks.NewtonOnCompoundSubCollisionAABBOverlap;
import crab.newton.callbacks.NewtonOnContactGeneration;
import crab.newton.callbacks.NewtonOnJointDeserializationCallback;
import crab.newton.callbacks.NewtonOnJointSerializationCallback;
import crab.newton.callbacks.NewtonPostUpdateCallback;
import crab.newton.callbacks.NewtonSerializeCallback;
import crab.newton.callbacks.NewtonWorldDestroyListenerCallback;
import crab.newton.callbacks.NewtonWorldDestructorCallback;
import crab.newton.callbacks.NewtonWorldListenerBodyDestroyCallback;
import crab.newton.callbacks.NewtonWorldListenerDebugCallback;
import crab.newton.callbacks.NewtonWorldRayFilterCallback;
import crab.newton.callbacks.NewtonWorldRayPrefilterCallback;
import crab.newton.callbacks.NewtonWorldUpdateListenerCallback;

import java.lang.foreign.*;
import java.nio.charset.StandardCharsets;

/**
 * @author Christopher
 */
public record NewtonWorld(MemorySegment address) {

	/**
	 * public constructor of the {@code NewtonWorld} class.
	 * This constructor is mainly meant to be used internally or with methods that
	 * return raw pointers to a NewtonWorld object. Users of this class should create a
	 * {@code NewtonWorld} with the {@code NewtonWorld.create()} method.
	 *
	 * @param address address of the NewtonWorld object
	 */
	public NewtonWorld {}

	/**
	 * Creates a NewtonWorld instance
	 *
	 * @return NewtonWorld
	 */
	public static NewtonWorld create() {
		return new NewtonWorld(Newton.NewtonCreate());
	}

	public NewtonCollision createBox(float dx, float dy, float dz, int shapeID, MemorySegment offsetMatrix) {
		if (offsetMatrix.byteSize() != Newton.MAT4F.byteSize()) {
			throw new RuntimeException("offsetMatrix incorrect size");
		}
		return new NewtonCollision(Newton.NewtonCreateBox(address, dx, dy, dz, shapeID, offsetMatrix));
	}

	/**
	 * Creates an instance of a box collision
	 *
	 * @param dx
	 * @param dy
	 * @param dz
	 * @param shapeID
	 * @param offsetMatrix
	 * @return
	 */
	public NewtonCollision createBox(float dx, float dy, float dz, int shapeID, float[] offsetMatrix) {
		if (offsetMatrix.length != 16) {
			throw new RuntimeException("offsetMatrix incorrect length");
		}
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrix = arena.allocateArray(Newton.C_FLOAT, offsetMatrix);
			return new NewtonCollision(Newton.NewtonCreateBox(address, dx, dy, dz, shapeID, matrix));
		}
	}

	public NewtonCollision createBox(float dx, float dy, float dz, int shapeID) {
		return new NewtonCollision(Newton.NewtonCreateBox(address, dx, dy, dz, shapeID, MemorySegment.NULL));
	}

	public NewtonCollision createCapsule(float radius0, float radius1, float height, int shapeID, MemorySegment offsetMatrix) {
		if (offsetMatrix.byteSize() != Newton.MAT4F.byteSize()) {
			throw new RuntimeException("offsetMatrix incorrect size");
		}
		return new NewtonCollision(Newton.NewtonCreateCapsule(address, radius0, radius1, height, shapeID, offsetMatrix));
	}

	/**
	 * @param radius0
	 * @param radius1
	 * @param height
	 * @param shapeID
	 * @param offsetMatrix
	 * @return
	 */
	public NewtonCollision createCapsule(float radius0, float radius1, float height, int shapeID, float[] offsetMatrix) {
		if (offsetMatrix.length != 16) {
			throw new RuntimeException("offsetMatrix incorrect length");
		}
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrix = arena.allocateArray(Newton.C_FLOAT, offsetMatrix);
			return new NewtonCollision(Newton.NewtonCreateCapsule(address, radius0, radius1, height, shapeID, matrix));
		}
	}

	public NewtonCollision createCapsule(float radius0, float radius1, float height, int shapeID) {
		return new NewtonCollision(Newton.NewtonCreateCapsule(address, radius0, radius1, height, shapeID, MemorySegment.NULL));
	}

	public NewtonCollision createChamferCylinder(float radius, float height, int shapeID, MemorySegment offsetMatrix) {
		if (offsetMatrix.byteSize() != Newton.MAT4F.byteSize()) {
			throw new RuntimeException("offsetMatrix incorrect size");
		}
		return new NewtonCollision(Newton.NewtonCreateChamferCylinder(address, radius, height, shapeID, offsetMatrix));
	}

	/**
	 * @param radius
	 * @param height
	 * @param shapeID
	 * @param offsetMatrix
	 * @return
	 */
	public NewtonCollision createChamferCylinder(float radius, float height, int shapeID, float[] offsetMatrix) {
		if (offsetMatrix.length != 16) {
			throw new RuntimeException("offsetMatrix incorrect length");
		}
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrix = arena.allocateArray(Newton.C_FLOAT, offsetMatrix);
			return new NewtonCollision(Newton.NewtonCreateChamferCylinder(address, radius, height, shapeID, matrix));
		}
	}

	public NewtonCollision createChamferCylinder(float radius, float height, int shapeID) {
		return new NewtonCollision(Newton.NewtonCreateChamferCylinder(address, radius, height, shapeID, MemorySegment.NULL));
	}

	public NewtonCollision createCompoundCollision(int shapeID) {
		return new NewtonCollision(Newton.NewtonCreateCompoundCollision(address, shapeID));
	}

	public NewtonCollision createCompoundCollision(NewtonMesh mesh, float hullTolerance, int shapeID, int subShapeID) {
		return new NewtonCollision(Newton.NewtonCreateCompoundCollisionFromMesh(address, mesh.address(), hullTolerance, shapeID, subShapeID));
	}

	public NewtonCollision createCone(float radius, float height, int shapeID, MemorySegment offsetMatrix) {
		if (offsetMatrix.byteSize() != Newton.MAT4F.byteSize()) {
			throw new RuntimeException("offsetMatrix incorrect size");
		}
		return new NewtonCollision(Newton.NewtonCreateCone(address, radius, height, shapeID, offsetMatrix));
	}

	public NewtonCollision createCone(float radius, float height, int shapeID, float[] offsetMatrix) {
		if (offsetMatrix.length != 16) {
			throw new RuntimeException("offsetMatrix incorrect length");
		}
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrix = arena.allocateArray(Newton.C_FLOAT, offsetMatrix);
			return new NewtonCollision(Newton.NewtonCreateCone(address, radius, height, shapeID, matrix));
		}
	}

	public NewtonCollision createCone(float radius, float height, int shapeID) {
		return new NewtonCollision(Newton.NewtonCreateCone(address, radius, height, shapeID, MemorySegment.NULL));
	}

	public NewtonCollision createConvexHull(int count, MemorySegment vertexCloud, int strideInBytes, float tolerance, int shapeID, MemorySegment offsetMatrix) {
		if (offsetMatrix.byteSize() != Newton.MAT4F.byteSize()) {
			throw new RuntimeException("offsetMatrix incorrect size");
		}
		return new NewtonCollision(Newton.NewtonCreateConvexHull(address, count, vertexCloud, strideInBytes, tolerance, shapeID, offsetMatrix));
	}

	public NewtonCollision createConvexHull(int count, MemorySegment vertexCloud, int strideInBytes, float tolerance, int shapeID) {
		return new NewtonCollision(Newton.NewtonCreateConvexHull(address, count, vertexCloud, strideInBytes, tolerance, shapeID, MemorySegment.NULL));
	}

	public NewtonCollision createConvexHull(int count, float[] vertexCloud, int strideInBytes, float tolerance, int shapeID, float[] offsetMatrix) {
		if (offsetMatrix.length != 16) {
			throw new RuntimeException("offsetMatrix incorrect length");
		}
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrix = arena.allocateArray(Newton.C_FLOAT, offsetMatrix);
			MemorySegment vertCloud = arena.allocateArray(Newton.C_FLOAT, vertexCloud);
			return new NewtonCollision(Newton.NewtonCreateConvexHull(address, count, vertCloud, strideInBytes, tolerance, shapeID, matrix));
		}
	}

	public NewtonCollision createConvexHull(int count, float[] vertexCloud, int strideInBytes, float tolerance, int shapeID) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment vertCloud = arena.allocateArray(Newton.C_FLOAT, vertexCloud);
			return new NewtonCollision(Newton.NewtonCreateConvexHull(address, count, vertCloud, strideInBytes, tolerance, shapeID, MemorySegment.NULL));
		}
	}

	public NewtonCollision createCylinder(float radio0, float radio1, float height, int shapeID, MemorySegment offsetMatrix) {
		if (offsetMatrix.byteSize() != Newton.MAT4F.byteSize()) {
			throw new RuntimeException("offsetMatrix incorrect size");
		}
		return new NewtonCollision(Newton.NewtonCreateCylinder(address, radio0, radio1, height, shapeID, offsetMatrix));
	}

	public NewtonCollision createCylinder(float radio0, float radio1, float height, int shapeID, float[] offsetMatrix) {
		if (offsetMatrix.length != 16) {
			throw new RuntimeException("offsetMatrix incorrect length");
		}
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrix = arena.allocateArray(Newton.C_FLOAT, offsetMatrix);
			return new NewtonCollision(Newton.NewtonCreateCylinder(address, radio0, radio1, height, shapeID, matrix));
		}
	}

	public NewtonCollision createCylinder(float radio0, float radio1, float height, int shapeID) {
		return new NewtonCollision(Newton.NewtonCreateCylinder(address, radio0, radio1, height, shapeID, MemorySegment.NULL));
	}

	public NewtonCollision createHeightField(int width, int height, int gridsDiagonals, int elevationDataType, MemorySegment elevationMap, MemorySegment attributeMap, float verticalScale, float horizontalScale_x, float horizontalScale_z, int shapeID) {
		return new NewtonCollision(Newton.NewtonCreateHeightFieldCollision(address, width, height, gridsDiagonals, elevationDataType, elevationMap, attributeMap, verticalScale, horizontalScale_x, horizontalScale_z, shapeID));
	}

	public NewtonCollision createHeightField(int width, int height, int gridsDiagonals, float[] elevationMap, char[] attributeMap, float verticalScale, float horizontalScale_x, float horizontalScale_z, int shapeID) {
		int elevationDataType = 0;
		try (Arena arena = Arena.openConfined()) {
			MemorySegment elevationMapSegment = arena.allocateArray(Newton.C_FLOAT, elevationMap);
			MemorySegment attributeSeg = arena.allocateArray(Newton.C_CHAR, new String(attributeMap).getBytes(StandardCharsets.UTF_8));
			return new NewtonCollision(Newton.NewtonCreateHeightFieldCollision(address, width, height, gridsDiagonals, elevationDataType, elevationMapSegment, attributeSeg, verticalScale, horizontalScale_x, horizontalScale_z, shapeID));
		}
	}

	public NewtonCollision createHeightField(int width, int height, int gridsDiagonals, short[] elevationMap, char[] attributeMap, float verticalScale, float horizontalScale_x, float horizontalScale_z, int shapeID) {
		int elevationDataType = 1;
		try (Arena arena = Arena.openConfined()) {
			MemorySegment elevationMapSegment = arena.allocateArray(Newton.C_SHORT, elevationMap);
			MemorySegment attributeSeg = arena.allocateArray(Newton.C_CHAR, new String(attributeMap).getBytes(StandardCharsets.UTF_8));
			return new NewtonCollision(Newton.NewtonCreateHeightFieldCollision(address, width, height, gridsDiagonals, elevationDataType, elevationMapSegment, attributeSeg, verticalScale, horizontalScale_x, horizontalScale_z, shapeID));
		}
	}

	public NewtonCollision createNullCollision() {
		return new NewtonCollision(Newton.NewtonCreateNull(address));
	}

	public NewtonCollision createSceneCollision(int shapeID) {
		return new NewtonCollision(Newton.NewtonCreateSceneCollision(address, shapeID));
	}

	public NewtonCollision createSphere(float radius, int shapeID, MemorySegment offsetMatrix) {
		if (offsetMatrix.byteSize() != Newton.MAT4F.byteSize()) {
			throw new RuntimeException("offsetMatrix incorrect size");
		}
		return new NewtonCollision(Newton.NewtonCreateSphere(address, radius, shapeID, offsetMatrix));
	}

	public NewtonCollision createSphere(float radius, int shapeID, float[] offsetMatrix) {
		if (offsetMatrix.length != 16) {
			throw new RuntimeException("offsetMatrix incorrect length");
		}
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrix = arena.allocateArray(Newton.C_FLOAT, offsetMatrix);
			return new NewtonCollision(Newton.NewtonCreateSphere(address, radius, shapeID, matrix));
		}
	}

	public NewtonCollision createSphere(float radius, int shapeID) {
		return new NewtonCollision(Newton.NewtonCreateSphere(address, radius, shapeID, MemorySegment.NULL));
	}

	public NewtonCollision createTreeCollision(int shapeID) {
		return new NewtonCollision(Newton.NewtonCreateTreeCollision(address, shapeID));
	}

	public NewtonCollision createTreeCollisionFromMesh(NewtonMesh mesh, int shapeID) {
		return new NewtonCollision(Newton.NewtonCreateTreeCollisionFromMesh(address, mesh.address(), shapeID));
	}

	public NewtonCollisionAggregate createCollisionAggregate() {
		return new NewtonCollisionAggregate(Newton.NewtonCollisionAggregateCreate(address));
	}

	public NewtonBody createAsymetricDynamicBody(NewtonCollision collision, MemorySegment matrix) {
		if (matrix.byteSize() != Newton.MAT4F.byteSize()) {
			throw new RuntimeException("offsetMatrix incorrect size");
		}
		return new NewtonBody(Newton.NewtonCreateAsymetricDynamicBody(address, collision.address(), matrix));
	}

	public NewtonBody createAsymetricDynamicBody(NewtonCollision collision, float[] matrix) {
		if (matrix.length != 16) {
			throw new RuntimeException("matrix incorrect length");
		}
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocateArray(Newton.C_FLOAT, matrix);
			return new NewtonBody(Newton.NewtonCreateAsymetricDynamicBody(address, collision.address(), matrixSeg));
		}
	}

	public NewtonBody createAsymetricDynamicBody(MemorySegment matrix) {
		if (matrix.byteSize() != Newton.MAT4F.byteSize()) {
			throw new RuntimeException("offsetMatrix incorrect size");
		}
		return new NewtonBody(Newton.NewtonCreateAsymetricDynamicBody(address, MemorySegment.NULL, matrix));
	}

	public NewtonBody createAsymetricDynamicBody(float[] matrix) {
		if (matrix.length != 16) {
			throw new RuntimeException("matrix incorrect length");
		}
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocateArray(Newton.C_FLOAT, matrix);
			return new NewtonBody(Newton.NewtonCreateAsymetricDynamicBody(address, MemorySegment.NULL, matrixSeg));
		}
	}

	public NewtonBody createDynamicBody(NewtonCollision collision, MemorySegment matrix) {
		if (matrix.byteSize() != Newton.MAT4F.byteSize()) {
			throw new RuntimeException("offsetMatrix incorrect size");
		}
		return new NewtonBody(Newton.NewtonCreateDynamicBody(address, collision.address(), matrix));
	}

	public NewtonBody createDynamicBody(NewtonCollision collision, float[] matrix) {
		if (matrix.length != 16) {
			throw new RuntimeException("matrix incorrect length");
		}
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocateArray(Newton.C_FLOAT, matrix);
			return new NewtonBody(Newton.NewtonCreateDynamicBody(address, collision.address(), matrixSeg));
		}
	}

	public NewtonBody createDynamicBody(MemorySegment matrix) {
		if (matrix.byteSize() != Newton.MAT4F.byteSize()) {
			throw new RuntimeException("offsetMatrix incorrect size");
		}
		return new NewtonBody(Newton.NewtonCreateDynamicBody(address, MemorySegment.NULL, matrix));
	}

	public NewtonBody createDynamicBody(float[] matrix) {
		if (matrix.length != 16) {
			throw new RuntimeException("matrix incorrect length");
		}
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocateArray(Newton.C_FLOAT, matrix);
			return new NewtonBody(Newton.NewtonCreateDynamicBody(address, MemorySegment.NULL, matrixSeg));
		}
	}

	public NewtonBody createKinematicBody(NewtonCollision collision, MemorySegment matrix) {
		if (matrix.byteSize() != Newton.MAT4F.byteSize()) {
			throw new RuntimeException("offsetMatrix incorrect size");
		}
		return new NewtonBody(Newton.NewtonCreateKinematicBody(address, collision.address(), matrix));
	}

	public NewtonBody createKinematicBody(NewtonCollision collision, float[] matrix) {
		if (matrix.length != 16) {
			throw new RuntimeException("matrix incorrect length");
		}
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocateArray(Newton.C_FLOAT, matrix);
			return new NewtonBody(Newton.NewtonCreateKinematicBody(address, collision.address(), matrixSeg));
		}
	}

	public NewtonBody createKinematicBody(MemorySegment matrix) {
		if (matrix.byteSize() != Newton.MAT4F.byteSize()) {
			throw new RuntimeException("offsetMatrix incorrect size");
		}
		return new NewtonBody(Newton.NewtonCreateKinematicBody(address, MemorySegment.NULL, matrix));
	}

	public NewtonBody createKinematicBody(float[] matrix) {
		if (matrix.length != 16) {
			throw new RuntimeException("matrix incorrect length");
		}
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSeg = arena.allocateArray(Newton.C_FLOAT, matrix);
			return new NewtonBody(Newton.NewtonCreateKinematicBody(address, MemorySegment.NULL, matrixSeg));
		}
	}

	public NewtonMesh createMesh() {
		return new NewtonMesh(Newton.NewtonMeshCreate(address));
	}

	public NewtonMesh createConvexHullMesh(int count, MemorySegment vertexCloud, int strideInBytes, float tolerance) {
		return new NewtonMesh(Newton.NewtonMeshCreateConvexHull(address, count, vertexCloud, strideInBytes, tolerance));
	}

	public NewtonMesh createConvexHullMesh(int count, float[] vertexCloud, int strideInBytes, float tolerance) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment vertCloud = arena.allocateArray(Newton.C_FLOAT, vertexCloud);
			return new NewtonMesh(Newton.NewtonMeshCreateConvexHull(address, count, vertCloud, strideInBytes, tolerance));
		}
	}

	public NewtonMesh createVoronoiConvexDecomposition(int count, MemorySegment vertexCloud, int strideInBytes, int materialID, MemorySegment textureMatrix) {
		return new NewtonMesh(Newton.NewtonMeshCreateVoronoiConvexDecomposition(address, count, vertexCloud, strideInBytes, materialID, textureMatrix));
	}

	public NewtonMesh createVoronoiConvexDecomposition(int count, float[] vertexCloud, int strideInBytes, int materialID, float[] textureMatrix) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment vertCloud = arena.allocateArray(Newton.C_FLOAT, vertexCloud);
			MemorySegment textMatrix = arena.allocateArray(Newton.C_FLOAT, textureMatrix);
			return new NewtonMesh(Newton.NewtonMeshCreateVoronoiConvexDecomposition(address, count, vertCloud, strideInBytes, materialID, textMatrix));
		}
	}

	public NewtonMesh createMeshFromSerialization(NewtonDeserializeCallback deserializFunc, MemorySegment serializeHandle, SegmentScope scope) {
		MemorySegment func = NewtonDeserializeCallback.allocate(deserializFunc, scope);
		return new NewtonMesh(Newton.NewtonMeshCreateFromSerialization(address, func, serializeHandle));
	}

	public NewtonMesh loadTetrahedraMesh(String filename) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment cString = arena.allocateUtf8String(filename);
			return new NewtonMesh(Newton.NewtonMeshLoadTetrahedraMesh(address, cString));
		}
	}

	/**
	 * Gets the version of Newton
	 *
	 * @return Newton API Version
	 */
	public static int getWorldVersion() {
		return Newton.NewtonWorldGetVersion();
	}

	/**
	 * Gets the size of floats in Newton library
	 *
	 * @return size of floats in bytes
	 */
	public static int getFloatSizes() {
		return Newton.NewtonWorldFloatSize();
	}

	/**
	 * Gets the amount of memory used by Newton
	 *
	 * @return amount of memory used in bytes
	 */
	public static int getMemoryUsed() {
		return Newton.NewtonGetMemoryUsed();
	}

	/**
	 * Set the memory system for Newton library
	 *
	 * @param alloc - allocation function
	 * @param free  - deallocation function
	 * @param scope - ResourceScope for allocating upcall stubs
	 */
	public static void setMemorySystem(NewtonAllocMemory alloc, NewtonFreeMemory free, SegmentScope scope) {
		MemorySegment allocFunc = NewtonAllocMemory.allocate(alloc, scope);
		MemorySegment freeFunc = NewtonFreeMemory.allocate(free, scope);
		Newton.NewtonSetMemorySystem(allocFunc, freeFunc);
	}

	/**
	 * Allocates memory with Newton allocator
	 *
	 * @param sizeInBytes - size in bytes of memory to be allocated
	 * @return MemorySegment to the allocated memory
	 */
	public static MemorySegment newtonAlloc(int sizeInBytes) {
		return Newton.NewtonAlloc(sizeInBytes);
	}

	/**
	 * Allocates memory with Newton allocator
	 *
	 * @param sizeInBytes - size in bytes of memory to be allocated
	 * @param scope       - ResourceScope for the returned MemorySegment
	 * @return MemorySegment representing the allocated memory
	 */
	public static MemorySegment newtonAlloc(int sizeInBytes, SegmentScope scope) {
		return MemorySegment.ofAddress(Newton.NewtonAlloc(sizeInBytes).address(), sizeInBytes, scope);
	}

	/**
	 * Allocates memory with Newton allocator and given MemoryLayout
	 *
	 * @param layout - MemoryLayout to be allocated
	 * @return MemorySegment to the allocated memory
	 */
	public static MemorySegment newtonAlloc(MemoryLayout layout) {
		return Newton.NewtonAlloc((int) layout.byteSize());
	}

	/**
	 * Allocates memory with Newton allocator and given MemoryLayout
	 *
	 * @param layout - MemoryLayout to be allocated
	 * @param scope  - ResourceScope for the returned MemorySegment
	 * @return MemorySegment representing the allocated memory
	 */
	public static MemorySegment newtonAlloc(MemoryLayout layout, SegmentScope scope) {
		return MemorySegment.ofAddress(Newton.NewtonAlloc((int) layout.byteSize()).address(), layout.byteSize(), scope);
	}

	/**
	 * Frees memory allocated by Newton
	 *
	 * @param ptr - pointer to the memory create by Newton
	 */
	public static void newtonFree(MemorySegment ptr) {
		Newton.NewtonFree(ptr);
	}

	/**
	 * Destroys the current Newton object.
	 */
	public void destroy() {
		Newton.NewtonDestroy(address);
	}

	public NewtonPostUpdateCallback getPostUpdateCallback(SegmentScope scope) {
		return NewtonPostUpdateCallback.ofAddress(Newton.NewtonGetPostUpdateCallback(address), scope);
	}

	public void setPostUpdateCallback(NewtonPostUpdateCallback callback, SegmentScope scope) {
		MemorySegment callbackFunc = NewtonPostUpdateCallback.allocate(callback, scope);
		Newton.NewtonSetPostUpdateCallback(address, callbackFunc);
	}

	public void loadPlugins(String pluginPath) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment path = arena.allocateUtf8String(pluginPath);
			Newton.NewtonLoadPlugins(address, path);
		}
	}

	public void unloadPlugins() {
		Newton.NewtonUnloadPlugins(address);
	}

	public MemorySegment getCurrentPlugin() {
		return Newton.NewtonCurrentPlugin(address);
	}

	public MemorySegment getFirstPlugin() {
		return Newton.NewtonGetFirstPlugin(address);
	}

	public MemorySegment getNextPlugin(MemorySegment curPlugin) {
		return Newton.NewtonGetNextPlugin(address, curPlugin);
	}

	public MemorySegment getPreferedPlugin() {
		return Newton.NewtonGetPreferedPlugin(address);
	}

	public float getContactMergeTolerance() {
		return Newton.NewtonGetContactMergeTolerance(address);
	}

	public void setContactMergeTolerance(float tolerance) {
		Newton.NewtonSetContactMergeTolerance(address, tolerance);
	}

	public void invalidateCache() {
		Newton.NewtonInvalidateCache(address);
	}

	public void setSolverIterations(int model) {
		Newton.NewtonSetSolverIterations(address, model);
	}

	public int getSolverIterations() {
		return Newton.NewtonGetSolverIterations(address);
	}

	public void setParallelSolverOnLargeIsland(int mode) {
		Newton.NewtonSetParallelSolverOnLargeIsland(address, mode);
	}

	public int getParallelSolverOnLargeIsland() {
		return Newton.NewtonGetParallelSolverOnLargeIsland(address);
	}

	public int getBroadphaseAlgorithm() {
		return Newton.NewtonGetBroadphaseAlgorithm(address);
	}

	public void selectBroadphaseAlgorithm(int algorithmType) {
		Newton.NewtonSelectBroadphaseAlgorithm(address, algorithmType);
	}

	public void resetBroadphase() {
		Newton.NewtonResetBroadphase(address);
	}

	public void update(float timestep) {
		Newton.NewtonUpdate(address, timestep);
	}

	public void updateAsync(float timestep) {
		Newton.NewtonUpdateAsync(address, timestep);
	}

	public void waitForUpdateToFinish() {
		Newton.NewtonWaitForUpdateToFinish(address);
	}

	public int getNumberOfSubsteps() {
		return Newton.NewtonGetNumberOfSubsteps(address);
	}

	public void setNumberOfSubsteps(int substeps) {
		Newton.NewtonSetNumberOfSubsteps(address, substeps);
	}

	public float getLastUpdateTime() {
		return Newton.NewtonGetLastUpdateTime(address);
	}

	public void serializeToFile(String filename, NewtonOnBodySerializationCallback bodyCallback, MemorySegment bodyUserData,
								SegmentScope scope) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment filePath = arena.allocateUtf8String(filename);
			MemorySegment callbackFunc = NewtonOnBodySerializationCallback.allocate(bodyCallback, scope);
			Newton.NewtonSerializeToFile(address, filePath, callbackFunc, bodyUserData);
		}
	}

	public void deserializeFromFile(String filename, NewtonOnBodyDeserializationCallback bodyCallback, MemorySegment bodyUserData,
									SegmentScope scope) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment filePath = arena.allocateUtf8String(filename);
			MemorySegment callbackFunc = NewtonOnBodyDeserializationCallback.allocate(bodyCallback, scope);
			Newton.NewtonDeserializeFromFile(address, filePath, callbackFunc, bodyUserData);
		}
	}

	public void serializeScene(NewtonOnBodySerializationCallback bodyCallback, MemorySegment bodyUserData, NewtonSerializeCallback serializeCallback, MemorySegment serializeHandle,
							   SegmentScope scope) {
		MemorySegment bodyFunc = NewtonOnBodySerializationCallback.allocate(bodyCallback, scope);
		MemorySegment serializeFunc = NewtonSerializeCallback.allocate(serializeCallback, scope);
		Newton.NewtonSerializeScene(address, bodyFunc, bodyUserData, serializeFunc, serializeHandle);
	}

	public void deserializeScene(NewtonOnBodyDeserializationCallback bodyCallback, MemorySegment bodyUserData, NewtonDeserializeCallback serializeCallback, MemorySegment serializeHandle,
								 SegmentScope scope) {
		MemorySegment bodyFunc = NewtonOnBodyDeserializationCallback.allocate(bodyCallback, scope);
		MemorySegment deserializeFunc = NewtonDeserializeCallback.allocate(serializeCallback, scope);
		Newton.NewtonDeserializeScene(address, bodyFunc, bodyUserData, deserializeFunc, serializeHandle);
	}

	public NewtonBody findSerializedBody(int bodySerializedID) {
		MemorySegment body = Newton.NewtonFindSerializedBody(address, bodySerializedID);
		int bodyType = Newton.NewtonBodyGetType(body);
		return new NewtonBody(body, bodyType);
	}

	public void setJointSerializationCallbacks(NewtonOnJointSerializationCallback serializeJoint, NewtonOnJointDeserializationCallback deserializeJoint,
											   SegmentScope scope) {
		MemorySegment serializeFunc = NewtonOnJointSerializationCallback.allocate(serializeJoint, scope);
		MemorySegment deserializeFunc = NewtonOnJointDeserializationCallback.allocate(deserializeJoint, scope);
		Newton.NewtonSetJointSerializationCallbacks(address, serializeFunc, deserializeFunc);
	}

	public JointSerializationCallbacks getJointSerializationCallbacks(SegmentScope scope) {
		MemorySegment serializePtr = MemorySegment.NULL;
		MemorySegment deserializePtr = MemorySegment.NULL;
		Newton.NewtonGetJointSerializationCallbacks(address, serializePtr, deserializePtr);
		NewtonOnJointSerializationCallback serializeCallback = NewtonOnJointSerializationCallback.ofAddress(serializePtr, scope);
		NewtonOnJointDeserializationCallback deserializeCallback = NewtonOnJointDeserializationCallback.ofAddress(deserializePtr, scope);
		return new JointSerializationCallbacks(serializeCallback, deserializeCallback);
	}

	public void lockCriticalSection(int threadIndex) {
		Newton.NewtonWorldCriticalSectionLock(address, threadIndex);
	}

	public void unlockCriticalSection() {
		Newton.NewtonWorldCriticalSectionUnlock(address);
	}

	public void setThreadCount(int threads) {
		Newton.NewtonSetThreadsCount(address, threads);
	}

	public int getThreadCount() {
		return Newton.NewtonGetThreadsCount(address);
	}

	public int getMaxThreadCount() {
		return Newton.NewtonGetMaxThreadsCount(address);
	}

	public void dispatchThreadJob(NewtonJobTask task, MemorySegment userData, String functionName, SegmentScope scope) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment function_name = arena.allocateUtf8String(functionName);
			MemorySegment taskFunc = NewtonJobTask.allocate(task, scope);
			Newton.NewtonDispachThreadJob(address, taskFunc, userData, function_name);
		}
	}

	public void syncThreadJobs() {
		Newton.NewtonSyncThreadJobs(address);
	}

	public void setIslandUpdateEvent(NewtonIslandUpdate islandUpdate, SegmentScope scope) {
		MemorySegment islandUpdateFunc = NewtonIslandUpdate.allocate(islandUpdate, scope);
		Newton.NewtonSetIslandUpdateEvent(address, islandUpdateFunc);
	}

	public void forEachJoint(NewtonJointIterator callback, MemorySegment userData, SegmentScope scope) {
		MemorySegment callbackFunc = NewtonJointIterator.allocate(callback, scope);
		Newton.NewtonWorldForEachJointDo(address, callbackFunc, userData);
	}

	public void forEachBodyInAABB(float[] p0, float[] p1, NewtonBodyIterator callback, MemorySegment userData, SegmentScope scope) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment p0Segment = arena.allocateArray(Newton.C_FLOAT, p0);
			MemorySegment p1Segment = arena.allocateArray(Newton.C_FLOAT, p1);
			MemorySegment callbackFunc = NewtonBodyIterator.allocate(callback, scope);
			Newton.NewtonWorldForEachBodyInAABBDo(address, p0Segment, p1Segment, callbackFunc, userData);
		}
	}

	public void setUserData(MemorySegment userData) {
		Newton.NewtonWorldSetUserData(address, userData);
	}

	public MemorySegment getUserData() {
		return Newton.NewtonWorldGetUserData(address);
	}

	public MemorySegment addListener(String nameId, MemorySegment listenerUserData) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment nameIdSegment = arena.allocateUtf8String(nameId);
			return Newton.NewtonWorldAddListener(address, nameIdSegment, listenerUserData);
		}
	}

	public MemorySegment getListener(String nameId) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment nameIdSegment = arena.allocateUtf8String(nameId);
			return Newton.NewtonWorldGetListener(address, nameIdSegment);
		}
	}

	public void listenerSetDebugCallback(MemorySegment listener, NewtonWorldListenerDebugCallback callback, SegmentScope scope) {
		MemorySegment callbackFunc = NewtonWorldListenerDebugCallback.allocate(callback, scope);
		Newton.NewtonWorldListenerSetDebugCallback(address, listener, callbackFunc);
	}

	public void listenerSetPostStepCallback(MemorySegment listener, NewtonWorldUpdateListenerCallback callback, SegmentScope scope) {
		MemorySegment callbackFunc = NewtonWorldUpdateListenerCallback.allocate(callback, scope);
		Newton.NewtonWorldListenerSetPostStepCallback(address, listener, callbackFunc);
	}

	public void listenerSetPreUpdateCallback(MemorySegment listener, NewtonWorldUpdateListenerCallback callback, SegmentScope scope) {
		MemorySegment callbackFunc = NewtonWorldUpdateListenerCallback.allocate(callback, scope);
		Newton.NewtonWorldListenerSetPreUpdateCallback(address, listener, callbackFunc);
	}

	public void listenerSetPostUpdateCallback(MemorySegment listener, NewtonWorldUpdateListenerCallback callback, SegmentScope scope) {
		MemorySegment callbackFunc = NewtonWorldUpdateListenerCallback.allocate(callback, scope);
		Newton.NewtonWorldListenerSetPostUpdateCallback(address, listener, callbackFunc);
	}

	public void listenerSetDestructorCallback(MemorySegment listener, NewtonWorldDestroyListenerCallback callback, SegmentScope scope) {
		MemorySegment callbackFunc = NewtonWorldDestroyListenerCallback.allocate(callback, scope);
		Newton.NewtonWorldListenerSetDestructorCallback(address, listener, callbackFunc);
	}

	public void listenerSetBodyDestroyCallback(MemorySegment listener, NewtonWorldListenerBodyDestroyCallback callback, SegmentScope scope) {
		MemorySegment callbackFunc = NewtonWorldListenerBodyDestroyCallback.allocate(callback, scope);
		Newton.NewtonWorldListenerSetBodyDestroyCallback(address, listener, callbackFunc);
	}

	public void listenerDebug(MemorySegment context) {
		Newton.NewtonWorldListenerDebug(address, context);
	}

	public MemorySegment getListenerUserData(MemorySegment listener) {
		return Newton.NewtonWorldGetListenerUserData(address, listener);
	}

	public NewtonWorldListenerBodyDestroyCallback listenerGetBodyDestroyCallback(MemorySegment listener, SegmentScope scope) {
		return NewtonWorldListenerBodyDestroyCallback.ofAddress(Newton.NewtonWorldListenerGetBodyDestroyCallback(address, listener), scope);
	}

	public void setDestructorCallback(NewtonWorldDestructorCallback destructor, SegmentScope scope) {
		MemorySegment destructorFunc = NewtonWorldDestructorCallback.allocate(destructor, scope);
		Newton.NewtonWorldSetDestructorCallback(address, destructorFunc);
	}

	public NewtonWorldDestructorCallback getDestructorCallback(SegmentScope scope) {
		return NewtonWorldDestructorCallback.ofAddress(Newton.NewtonWorldGetDestructorCallback(address), scope);
	}

	public void setCollisionConstructorDestructorCallback(NewtonCollisionCopyConstructionCallback constructor, NewtonCollisionDestructorCallback destructor, SegmentScope scope) {
		MemorySegment constructorFunc = NewtonCollisionCopyConstructionCallback.allocate(constructor, scope);
		MemorySegment destructorFunc = NewtonCollisionDestructorCallback.allocate(destructor, scope);
		Newton.NewtonWorldSetCollisionConstructorDestructorCallback(address, constructorFunc, destructorFunc);
	}

	public void setCreateDestroyContactCallback(NewtonCreateContactCallback createContact, NewtonDestroyContactCallback destroyContact, SegmentScope scope) {
		MemorySegment createFunc = NewtonCreateContactCallback.allocate(createContact, scope);
		MemorySegment destroyFunc = NewtonDestroyContactCallback.allocate(destroyContact, scope);
		Newton.NewtonWorldSetCreateDestroyContactCallback(address, createFunc, destroyFunc);
	}

	public void rayCast(MemorySegment p0, MemorySegment p1, NewtonWorldRayFilterCallback filter, MemorySegment userData, NewtonWorldRayPrefilterCallback prefilter, int threadIndex,
						SegmentScope scope) {
		MemorySegment filterFunc = NewtonWorldRayFilterCallback.allocate(filter, scope);
		MemorySegment preFilterFunc = NewtonWorldRayPrefilterCallback.allocate(prefilter, scope);
		Newton.NewtonWorldRayCast(address, p0, p1, filterFunc, userData, preFilterFunc, threadIndex);
	}

	public void rayCast(float[] p0, float[] p1, NewtonWorldRayFilterCallback filter, MemorySegment userData, NewtonWorldRayPrefilterCallback prefilter, int threadIndex,
						SegmentScope scope) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment p0Segment = arena.allocateArray(Newton.C_FLOAT, p0);
			MemorySegment p1Segment = arena.allocateArray(Newton.C_FLOAT, p1);
			MemorySegment filterFunc = NewtonWorldRayFilterCallback.allocate(filter, scope);
			MemorySegment preFilterFunc = NewtonWorldRayPrefilterCallback.allocate(prefilter, scope);
			Newton.NewtonWorldRayCast(address, p0Segment, p1Segment, filterFunc, userData, preFilterFunc, threadIndex);
		}
	}

	public int convexCast(MemorySegment matrix, MemorySegment target, NewtonCollision shape, MemorySegment param, MemorySegment userData, NewtonWorldRayPrefilterCallback prefilter,
						  MemorySegment info, int maxContactsCount, int threadIndex, SegmentScope scope) {
		MemorySegment prefilterFunc = NewtonWorldRayPrefilterCallback.allocate(prefilter, scope);
		return Newton.NewtonWorldConvexCast(address, matrix, target, shape.address(), param, userData, prefilterFunc, info, maxContactsCount, threadIndex);
	}

	public int convexCast(float[] matrix, float[] target, NewtonCollision shape, MemorySegment param, MemorySegment userData, NewtonWorldRayPrefilterCallback prefilter,
						  MemorySegment info, int maxContactsCount, int threadIndex, SegmentScope scope) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrixSegment = arena.allocateArray(Newton.C_FLOAT, matrix);
			MemorySegment targetSegment = arena.allocateArray(Newton.C_FLOAT, target);
			MemorySegment prefilterFunc = NewtonWorldRayPrefilterCallback.allocate(prefilter, scope);
			return Newton.NewtonWorldConvexCast(address, matrixSegment, targetSegment, shape.address(), param, userData, prefilterFunc, info, maxContactsCount, threadIndex);
		}
	}

	public int getBodyCount() {
		return Newton.NewtonWorldGetBodyCount(address);
	}

	public int getConstraintCount() {
		return Newton.NewtonWorldGetConstraintCount(address);
	}

	public int createMaterialGroupID() {
		return Newton.NewtonMaterialCreateGroupID(address);
	}

	public int getDefaultMaterialGroupID() {
		return Newton.NewtonMaterialGetDefaultGroupID(address);
	}

	public void destroyAllMaterialGroupIDs() {
		Newton.NewtonMaterialDestroyAllGroupID(address);
	}

	public MemorySegment getMaterialUserData(int id0, int id1) {
		return Newton.NewtonMaterialGetUserData(address, id0, id1);
	}

	public void setMaterialSurfaceThickness(int id0, int id1, float thickness) {
		Newton.NewtonMaterialSetSurfaceThickness(address, id0, id1, thickness);
	}

	public void setMaterialCallbackUserData(int id0, int id1, MemorySegment userData) {
		Newton.NewtonMaterialSetCallbackUserData(address, id0, id1, userData);
	}

	public void setMaterialContactGenerationCallback(int id0, int id1, NewtonOnContactGeneration contactGeneration, SegmentScope scope) {
		MemorySegment contactFunc = NewtonOnContactGeneration.allocate(contactGeneration, scope);
		Newton.NewtonMaterialSetContactGenerationCallback(address, id0, id1, contactFunc);
	}

	public void setMaterialCompoundCollisionCallback(int id0, int id1, NewtonOnCompoundSubCollisionAABBOverlap compoundAabbOverlap, SegmentScope scope) {
		MemorySegment overLapFunc = NewtonOnCompoundSubCollisionAABBOverlap.allocate(compoundAabbOverlap, scope);
		Newton.NewtonMaterialSetCompoundCollisionCallback(address, id0, id1, overLapFunc);
	}

	public void setMaterialCollisionCallback(int id0, int id1, NewtonOnAABBOverlap aabbOverlap, NewtonContactsProcess process, SegmentScope scope) {
		MemorySegment overlapFunc = NewtonOnAABBOverlap.allocate(aabbOverlap, scope);
		MemorySegment processFunc = NewtonContactsProcess.allocate(process, scope);
		Newton.NewtonMaterialSetCollisionCallback(address, id0, id1, overlapFunc, processFunc);
	}

	public void setMaterialDefaultSoftness(int id0, int id1, float softness) {
		Newton.NewtonMaterialSetDefaultSoftness(address, id0, id1, softness);
	}

	public void setMaterialDefaultElasticity(int id0, int id1, float elasticCoef) {
		Newton.NewtonMaterialSetDefaultElasticity(address, id0, id1, elasticCoef);
	}

	public void setMaterialDefaultCollidable(int id0, int id1, int state) {
		Newton.NewtonMaterialSetDefaultCollidable(address, id0, id1, state);
	}

	public void setMaterialDefaultFriction(int id0, int id1, float staticFriction, float kineticFriction) {
		Newton.NewtonMaterialSetDefaultFriction(address, id0, id1, staticFriction, kineticFriction);
	}

	public void resetMaterialJointIntraJointCollision(int id0, int id1) {
		Newton.NewtonMaterialJointResetIntraJointCollision(address, id0, id1);
	}

	public void resetMaterialJointSelftJointCollision(int id0, int id1) {
		Newton.NewtonMaterialJointResetSelftJointCollision(address, id0, id1);
	}

	public NewtonMaterial getFirstMaterial() {
		return new NewtonMaterial(Newton.NewtonWorldGetFirstMaterial(address));
	}

	public NewtonMaterial getNextMaterial(NewtonMaterial material) {
		MemorySegment materialPtr = Newton.NewtonWorldGetNextMaterial(address, material.address());
		return materialPtr.equals(MemorySegment.NULL) ? null : new NewtonMaterial(materialPtr);
	}

	public NewtonBody getFirstBody() {
		return new NewtonBody(Newton.NewtonWorldGetFirstBody(address));
	}

	public NewtonBody getNextBody(NewtonBody body) {
		MemorySegment bodyPtr = Newton.NewtonWorldGetNextBody(address, body.address());
		return bodyPtr.equals(MemorySegment.NULL) ? null : new NewtonBody(bodyPtr);
	}

	public NewtonCollision createCollisionFromSerialization(NewtonDeserializeCallback deserializeFunction, MemorySegment serializeHandle, SegmentScope scope) {
		MemorySegment deserializeFunc = NewtonDeserializeCallback.allocate(deserializeFunction, scope);
		return new NewtonCollision(Newton.NewtonCreateCollisionFromSerialization(address, deserializeFunc, serializeHandle));
	}

	public void serializeCollision(NewtonCollision collision, NewtonSerializeCallback serializeFunction, MemorySegment serializeHandle, SegmentScope scope) {
		MemorySegment serializeFunc = NewtonSerializeCallback.allocate(serializeFunction, scope);
		Newton.NewtonCollisionSerialize(address, collision.address(), serializeFunc, serializeHandle);
	}

	public void destroyAllBodies() {
		Newton.NewtonDestroyAllBodies(address);
	}

	public record JointSerializationCallbacks(NewtonOnJointSerializationCallback serializeCallback,
											  NewtonOnJointDeserializationCallback deserializeCallback) {
	}
}
