package crab.newton;

import crab.newton.callbacks.NewtonDeserializeCallback;
import crab.newton.callbacks.NewtonReportProgress;
import crab.newton.callbacks.NewtonSerializeCallback;
import crab.newton.internal.*;
import java.lang.foreign.*;

public class NewtonMesh {
	
	protected final MemoryAddress address;
	
	protected NewtonMesh(MemoryAddress address) {
		this.address = address;
	}
	
	/**
	 * Creates a new {@code NewtonMesh}
	 * @param world - {@code NewtonWorld} to allocate NewtonMesh
	 * @return new {@code NewtonMesh}
	 */
	public static NewtonMesh create(NewtonWorld world) {
		return new NewtonMesh(Newton_h.NewtonMeshCreate(world.address));
	}
	
	/**
	 * Create a new {@code NewtonMesh} from an existing mesh
	 * @param mesh - {@code NewtonMesh} to be copied
	 * @return new {@code NewtonMesh}
	 */
	public static NewtonMesh createFromMesh(NewtonMesh mesh) {
		return new NewtonMesh(Newton_h.NewtonMeshCreateFromMesh(mesh.address));
	}
	
	/**
	 * Creates a new {@code NewtonMesh} from a {@code NewtonCollision}
	 * @param collision - 
	 * @return
	 */
	public static NewtonMesh createFromCollision(NewtonCollision collision) {
		return new NewtonMesh(Newton_h.NewtonMeshCreateFromCollision(collision.address()));
	}
	
	public static NewtonMesh createTetrahedraIsoSurface(NewtonMesh mesh) {
		return new NewtonMesh(Newton_h.NewtonMeshCreateTetrahedraIsoSurface(mesh.address));
	}
	
	public static NewtonMesh createConvexHull(NewtonWorld world, int pointCount, float[] vertexCloud, int strideInBytes, float tolerance, 
			SegmentAllocator allocator) {
		MemorySegment vertCloud = allocator.allocateArray(Newton_h.C_FLOAT, vertexCloud);
		return new NewtonMesh(Newton_h.NewtonMeshCreateConvexHull(world.address, pointCount, vertCloud, strideInBytes, tolerance));
	}
	
	public static NewtonMesh createVoronoiConvexDecomposition(NewtonWorld world, int pointCount, float[] vertexCloud, int strideInBytes, int materialID, float[] textureMatrix,
			SegmentAllocator allocator) {
		MemorySegment vertCloud = allocator.allocateArray(Newton_h.C_FLOAT, vertexCloud);
		MemorySegment textMatrix = allocator.allocateArray(Newton_h.C_FLOAT, textureMatrix);
		return new NewtonMesh(Newton_h.NewtonMeshCreateVoronoiConvexDecomposition(world.address, pointCount, vertCloud, strideInBytes, materialID, textMatrix));
	}
	
	public static NewtonMesh createFromSerialization(NewtonWorld world, NewtonDeserializeCallback deserializFunc, Addressable serializeHandle, MemorySession session) {
		MemorySegment func = NewtonDeserializeCallback.allocate(deserializFunc, session);
		return new NewtonMesh(Newton_h.NewtonMeshCreateFromSerialization(world.address, func, serializeHandle));
	}
	
	public static NewtonMesh loadTetrahedraMesh(NewtonWorld world, String filename, SegmentAllocator allocator) {
		MemorySegment cString = allocator.allocateUtf8String(filename);
		return new NewtonMesh(Newton_h.NewtonMeshLoadTetrahedraMesh(world.address, cString));
	}
	
	public static void clearVertexFormat(MemorySegment meshVertexFormat) {
		Newton_h.NewtonMeshClearVertexFormat(meshVertexFormat);
	}
	
	public void destroy() {
		Newton_h.NewtonMeshDestroy(address);
	}
	
	public void serialize(NewtonSerializeCallback serializeFunction, Addressable serializeHandle, MemorySession session) {
		MemorySegment serializeFunc = NewtonSerializeCallback.allocate(serializeFunction, session);
		Newton_h.NewtonMeshSerialize(address, serializeFunc, serializeHandle);
	}
	
	public void flipWinding() {
		Newton_h.NewtonMeshFlipWinding(address);
	}
	
	public void applyTransform(float[] transform) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment matrix = session.allocateArray(Newton_h.C_FLOAT, transform);
			Newton_h.NewtonMeshApplyTransform(address, matrix);
		}
	}
	
	public float[] calculateOOBB() {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment oobb = session.allocateArray(Newton_h.C_FLOAT, Newton.MAT4F_VEC3F);
			Newton_h.NewtonMeshCalculateOOBB(address, 
					oobb.asSlice(0L, Newton_h.C_FLOAT.byteSize() * 16), 
					oobb.asSlice(64L, Newton_h.C_FLOAT.byteSize()), 
					oobb.asSlice(70L, Newton_h.C_FLOAT.byteSize()), 
					oobb.asSlice(74L, Newton_h.C_FLOAT.byteSize()));
			return oobb.toArray(Newton_h.C_FLOAT);
		}
	}
	
	public void calculateVertexNormals(float angleInRadians) {
		Newton_h.NewtonMeshCalculateVertexNormals(address, angleInRadians);
	}
	
	public void applySphericalMapping(int material, float[] alignMatrix) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment matrix = session.allocateArray(Newton_h.C_FLOAT, alignMatrix);
			Newton_h.NewtonMeshApplySphericalMapping(address, material, matrix);
		}
	}
	
	public void applyCylindricalMapping(int cylinderMaterial, int capMaterial, float[] alignMatrix) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment matrix = session.allocateArray(Newton_h.C_FLOAT, alignMatrix);
			Newton_h.NewtonMeshApplyCylindricalMapping(address, cylinderMaterial, capMaterial, matrix);
		}
	}
	
	public void applyBoxMapping(int frontMaterial, int sideMaterial, int topMaterial, float[] alignMatrix) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment matrix = session.allocateArray(Newton_h.C_FLOAT, alignMatrix);
			Newton_h.NewtonMeshApplyBoxMapping(address, frontMaterial, sideMaterial, topMaterial, matrix);
		}
	}
	
	public void applyAngleBasedMapping(int material, NewtonReportProgress reportPrograssCallback, Addressable reportPrgressUserData, float[] alignMatrix, MemorySession session) {
		try (MemorySession methodSession = MemorySession.openConfined()) {
			MemorySegment callback = NewtonReportProgress.allocate(reportPrograssCallback, session);
			MemorySegment matrix = methodSession.allocateArray(Newton_h.C_FLOAT, alignMatrix);
			Newton_h.NewtonMeshApplyAngleBasedMapping(address, material, callback, reportPrgressUserData, matrix);
		}
	}
	
	public void createTetrahedraLinearBlendSkinWeightsChannel(NewtonMesh skinMesh) {
		Newton_h.NewtonCreateTetrahedraLinearBlendSkinWeightsChannel(address, skinMesh.address);
	}
	
	public void optimize() {
		Newton_h.NewtonMeshOptimize(address);
	}
	
	public void optimizePoints() {
		Newton_h.NewtonMeshOptimizePoints(address);
	}
	
	public void optimizeVertex() {
		Newton_h.NewtonMeshOptimizeVertex(address);
	}
	
	public boolean isOpenMesh() {
		return Newton_h.NewtonMeshIsOpenMesh(address) == 1;
	}
	
	public void fixTJoints() {
		Newton_h.NewtonMeshFixTJoints(address);
	}
	
	public void polygonize() {
		Newton_h.NewtonMeshPolygonize(address);
	}
	
	public void triangulate() {
		Newton_h.NewtonMeshTriangulate(address);
	}
	
	public NewtonMesh convexMeshIntersection(NewtonMesh convexMesh) {
		return new NewtonMesh(Newton_h.NewtonMeshConvexMeshIntersection(address, convexMesh.address));
	}
	
	public void beginBuild() {
		Newton_h.NewtonMeshBeginBuild(address);
	}
	
	public void beginFace() {
		Newton_h.NewtonMeshBeginFace(address);
	}
	
	public void addPoint(double x, double y, double z) {
		Newton_h.NewtonMeshAddPoint(address, x, y, z);
	}
	
	public void addLayer(int layerIndex) {
		Newton_h.NewtonMeshAddLayer(address, layerIndex);
	}
	
	public void addMaterial(int materialIndex) {
		Newton_h.NewtonMeshAddMaterial(address, materialIndex);
	}
	
	public void addNormal(float x, float y, float z) {
		Newton_h.NewtonMeshAddNormal(address, x, y, z);
	}
	
	public void addBiNormal(float x, float y, float z) {
		Newton_h.NewtonMeshAddBinormal(address, x, y, z);
	}
	
	public void addUV0(float u, float v) {
		Newton_h.NewtonMeshAddUV0(address, u, v);
	}
	
	public void addUV1(float u, float v) {
		Newton_h.NewtonMeshAddUV1(address, u, v);
	}
	
	public void addVertexColor(float r, float g, float b, float a) {
		Newton_h.NewtonMeshAddVertexColor(address, r, g, b, a);
	}
	
	public void endFace() {
		Newton_h.NewtonMeshEndFace(address);
	}
	
	public void endBuild() {
		Newton_h.NewtonMeshEndBuild(address);
	}
	
	public void buildFromVertexListIndexList(MemorySegment meshVertexFormat) {
		Newton_h.NewtonMeshBuildFromVertexListIndexList(address, meshVertexFormat);
	}
	
	public int getPointCount() {
		return Newton_h.NewtonMeshGetPointCount(address);
	}
	
	public MemoryAddress getIndexToVertexMap() {
		return Newton_h.NewtonMeshGetIndexToVertexMap(address);
	}
	
	public int[] getIndexToVertexMap(int count) {
		MemoryAddress indexPtr = getIndexToVertexMap();
		try (MemorySession session = MemorySession.openConfined()) {
			return MemorySegment.ofAddress(indexPtr, Newton_h.C_INT.byteSize() * count, session).toArray(Newton_h.C_INT);
		}
	}
	
	public void getVertexDoubleChannel(int vertexStrideInBytes, MemorySegment vertexBuffer) {
		Newton_h.NewtonMeshGetVertexDoubleChannel(address, vertexStrideInBytes, vertexBuffer);
	}
	
	public double[] getVertexDoubleChannel(int vertexCount) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment vertexBuffer = session.allocateArray(Newton_h.C_DOUBLE, new double[vertexCount * 3]);
			getVertexDoubleChannel((int) (Newton_h.C_DOUBLE.byteSize() * 3), vertexBuffer);
			return vertexBuffer.toArray(Newton_h.C_DOUBLE);
		}
	}
	
	public void getVertexChannel(int vertexStrideInBytes, MemorySegment vertexBuffer) {
		Newton_h.NewtonMeshGetVertexChannel(address, vertexStrideInBytes, vertexBuffer);
	}
	
	public float[] getVertexChannel(int vertexCount) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment vertexBuffer = session.allocateArray(Newton_h.C_FLOAT, new float[vertexCount * 3]);
			getVertexChannel((int) (Newton_h.C_FLOAT.byteSize() * 3), vertexBuffer);
			return vertexBuffer.toArray(Newton_h.C_FLOAT);
		}
	}
	
	public void getNormalChannel(int vertexStrideInBytes, MemorySegment normalBuffer) {
		Newton_h.NewtonMeshGetNormalChannel(address, vertexStrideInBytes, normalBuffer);
	}
	
	public float[] getNormalChannel(int vertexCount) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment normalBuffer = session.allocateArray(Newton_h.C_FLOAT, new float[vertexCount * 3]);
			getNormalChannel((int) (Newton_h.C_FLOAT.byteSize() * 3), normalBuffer);
			return normalBuffer.toArray(Newton_h.C_FLOAT);
		}
	}
	
	public void getBiNormalChannel(int vertexStrideInBytes, MemorySegment biNormalBuffer) {
		Newton_h.NewtonMeshGetBinormalChannel(address, vertexStrideInBytes, biNormalBuffer);
	}
	
	public float[] getBiNormalChannel(int vertexCount) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment biNormalBuffer = session.allocateArray(Newton_h.C_FLOAT, new float[vertexCount * 3]);
			getBiNormalChannel((int) (Newton_h.C_FLOAT.byteSize() * 3), biNormalBuffer);
			return biNormalBuffer.toArray(Newton_h.C_FLOAT);
		}
	}
	
	public void getUV0Channel(int vertexStrideInBytes, MemorySegment uvBuffer) {
		Newton_h.NewtonMeshGetUV0Channel(address, vertexStrideInBytes, uvBuffer);
	}
	
	public float[] getUV0Channel(int vertexCount) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment uvBuffer = session.allocateArray(Newton_h.C_FLOAT, new float[vertexCount * 2]);
			getUV0Channel((int) (Newton_h.C_FLOAT.byteSize() * 2), uvBuffer);
			return uvBuffer.toArray(Newton_h.C_FLOAT);
		}
	}
	
	public void getUV1Channel(int vertexStrideInBytes, MemorySegment uvBuffer) {
		Newton_h.NewtonMeshGetUV1Channel(address, vertexStrideInBytes, uvBuffer);
	}
	
	public float[] getUV1Channel(int vertexCount) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment uvBuffer = session.allocateArray(Newton_h.C_FLOAT, new float[vertexCount * 2]);
			getUV1Channel((int) (Newton_h.C_FLOAT.byteSize() * 2), uvBuffer);
			return uvBuffer.toArray(Newton_h.C_FLOAT);
		}
	}
	
	public void getVertexColorChannel(int vertexStrideInBytes, MemorySegment colorBuffer) {
		Newton_h.NewtonMeshGetVertexColorChannel(address, vertexStrideInBytes, colorBuffer);
	}
	
	public float[] getVertexColorChannel(int vertexCount) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment colorBuffer = session.allocateArray(Newton_h.C_FLOAT, new float[vertexCount * 4]);
			getVertexColorChannel((int) (Newton_h.C_FLOAT.byteSize() * 4), colorBuffer);
			return colorBuffer.toArray(Newton_h.C_FLOAT);
		}
	}
	
	public boolean hasNormalChannel() {
		return Newton_h.NewtonMeshHasNormalChannel(address) == 1;
	}
	
	public boolean hasBiNormalChannel() {
		return Newton_h.NewtonMeshHasBinormalChannel(address) == 1;
	}
	
	public boolean hasUV0Channel() {
		return Newton_h.NewtonMeshHasUV0Channel(address) == 1;
	}
	
	public boolean hasUV1Channel() {
		return Newton_h.NewtonMeshHasUV1Channel(address) == 1;
	}
	
	public boolean hasVertexColorChannel() {
		return Newton_h.NewtonMeshHasVertexColorChannel(address) == 1;
	}
	
	public MemoryAddress beginHandle() {
		return Newton_h.NewtonMeshBeginHandle(address);
	}
	
	public void endHandle(MemoryAddress handle) {
		Newton_h.NewtonMeshEndHandle(address, handle);
	}
	
	public int firstMaterial(MemoryAddress handle) {
		return Newton_h.NewtonMeshFirstMaterial(address, handle);
	}
	
	public int nextMaterial(MemoryAddress handle, int materialID) {
		return Newton_h.NewtonMeshNextMaterial(address, handle, materialID);
	}
	
	public int materialGetMaterial(MemoryAddress handle, int materialID) {
		return Newton_h.NewtonMeshMaterialGetMaterial(address, handle, materialID);
	}
	
	public int materialGetIndexCount(MemoryAddress handle, int materialID) {
		return Newton_h.NewtonMeshMaterialGetIndexCount(address, handle, materialID);
	}
	
	public void materialGetIndexStream(MemoryAddress handle, int materialID, MemorySegment indexBuffer) {
		Newton_h.NewtonMeshMaterialGetIndexStream(address, handle, materialID, indexBuffer);
	}
	
	public int[] materialGetIndexStream(MemoryAddress handle, int materialID, int indexCount) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment indexStream = session.allocateArray(Newton_h.C_INT, new int[indexCount]);
			materialGetIndexStream(handle, materialID, indexStream);
			return indexStream.toArray(Newton_h.C_INT);
		}
	}
	
	public void materialGetIndexStreamShort(MemoryAddress handle, int materialID, MemorySegment indexBuffer) {
		Newton_h.NewtonMeshMaterialGetIndexStreamShort(address, handle, materialID, indexBuffer);
	}
	
	public short[] materialGetIndexStreamShort(MemoryAddress handle, int materialID, int indexCount) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment indexStream = session.allocateArray(Newton_h.C_SHORT, new short[indexCount]);
			materialGetIndexStreamShort(handle, materialID, indexStream);
			return indexStream.toArray(Newton_h.C_SHORT);
		}
	}
	
	public MemoryAddress createFirstSingleSegment() {
		return Newton_h.NewtonMeshCreateFirstSingleSegment(address);
	}
	
	public MemoryAddress createNextSingleSegment(MemoryAddress segment) {
		return Newton_h.NewtonMeshCreateNextSingleSegment(address, segment);
	}
	
	public MemoryAddress createFirstLayer() {
		return Newton_h.NewtonMeshCreateFirstLayer(address);
	}
	
	public MemoryAddress createNextLayer(MemoryAddress segment) {
		return Newton_h.NewtonMeshCreateNextLayer(address, segment);
	}
	
	public int getTotalFaceCount() {
		return Newton_h.NewtonMeshGetTotalFaceCount(address);
	}
	
	public int getTotalIndexCount() {
		return Newton_h.NewtonMeshGetTotalIndexCount(address);
	}
	
	public void getFaces(MemorySegment faceIndexCount, MemorySegment faceMaterial, MemorySegment faceIndices) {
		Newton_h.NewtonMeshGetFaces(address, faceIndexCount, faceMaterial, faceIndices);
	}
	
	public int getVertexCount() {
		return Newton_h.NewtonMeshGetVertexCount(address);
	}
	
	public int getVertexStrideInBytes() {
		return Newton_h.NewtonMeshGetVertexStrideInByte(address);
	}
	
	public MemoryAddress getVertexArray() {
		return Newton_h.NewtonMeshGetVertexArray(address);
	}
	
	public double[] getVertexArray(int vertexCount, int vertexStride) {
		MemoryAddress vertexBuffer = getVertexArray();
		try (MemorySession session = MemorySession.openConfined()) {
			return MemorySegment.ofAddress(vertexBuffer, Newton_h.C_DOUBLE.byteSize() * (vertexCount * vertexStride), session).toArray(Newton_h.C_DOUBLE);
		}
	}
	
	public int getVertexBaseCount() {
		return Newton_h.NewtonMeshGetVertexBaseCount(address);
	}
	
	public void setVertexBaseCount(int baseCount) {
		Newton_h.NewtonMeshSetVertexBaseCount(address, baseCount);
	}
	
	public MemoryAddress getFirstVertex() {
		return Newton_h.NewtonMeshGetFirstVertex(address);
	}
	
	public MemoryAddress getNextVertex(MemoryAddress vertex) {
		return Newton_h.NewtonMeshGetNextVertex(address, vertex);
	}
	
	public int getVertexIndex(MemoryAddress vertex) {
		return Newton_h.NewtonMeshGetVertexIndex(address, vertex);
	}
	
	public MemoryAddress getFirstPoint() {
		return Newton_h.NewtonMeshGetFirstPoint(address);
	}
	
	public MemoryAddress getNextPoint(MemoryAddress point) {
		return Newton_h.NewtonMeshGetNextPoint(address, point);
	}
	
	public int getPointIndex(MemoryAddress point) {
		return Newton_h.NewtonMeshGetPointIndex(address, point);
	}
	
	public int getVertexIndexFromPoint(MemoryAddress point) {
		return Newton_h.NewtonMeshGetVertexIndexFromPoint(address, point);
	}
	
	public MemoryAddress getFirstEdge() {
		return Newton_h.NewtonMeshGetFirstEdge(address);
	}
	
	public MemoryAddress getNextEdge(MemoryAddress edge) {
		return Newton_h.NewtonMeshGetNextEdge(address, edge);
	}
	
	public void getEdgeIndices(MemoryAddress edge, MemorySegment v0, MemorySegment v1) {
		Newton_h.NewtonMeshGetEdgeIndices(address, edge, v0, v1);
	}
	
	public int[] getEdgeIndices(MemoryAddress edge) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment indices = session.allocateArray(Newton_h.C_INT, new int[2]);
			getEdgeIndices(edge, indices.asSlice(0L, Newton_h.C_INT.byteSize()), indices.asSlice(4L, Newton_h.C_INT.byteSize()));
			return indices.toArray(Newton_h.C_INT);
		}
	}
	
	public MemoryAddress getFirstFace() {
		return Newton_h.NewtonMeshGetFirstFace(address);
	}
	
	public MemoryAddress getNextFace(MemoryAddress face) {
		return Newton_h.NewtonMeshGetNextFace(address, face);
	}
	
	public boolean isFaceOpen(MemoryAddress face) {
		return Newton_h.NewtonMeshIsFaceOpen(address, face) == 1;
	}
	
	public int getFaceMaterial(MemoryAddress face) {
		return Newton_h.NewtonMeshGetFaceMaterial(address, face);
	}
	
	public int getFaceIndexCount(MemoryAddress face) {
		return Newton_h.NewtonMeshGetFaceIndexCount(address, face);
	}
	
	public void getFaceIndices(MemoryAddress face, MemorySegment indicesBuffer) {
		Newton_h.NewtonMeshGetFaceIndices(address, face, indicesBuffer);
	}
	
	public int[] getFaceIndices(MemoryAddress face, int indiceCount) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment indices = session.allocateArray(Newton_h.C_INT, new int[indiceCount]);
			getFaceIndices(face, indices);
			return indices.toArray(Newton_h.C_INT);
		}
	}
	
	public void getFacePointIndices(MemoryAddress face, MemorySegment indicesBuffer) {
		Newton_h.NewtonMeshGetFacePointIndices(address, face, indicesBuffer);
	}
	
	public int[] getFacePointIndices(MemoryAddress face, int indiceCount) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment indices = session.allocateArray(Newton_h.C_INT, new int[indiceCount]);
			getFacePointIndices(face, indices);
			return indices.toArray(Newton_h.C_INT);
		}
	}
	
	public void calculateFaceNormals(MemoryAddress face, MemorySegment normalBuffer) {
		Newton_h.NewtonMeshCalculateFaceNormal(address, face, normalBuffer);
	}
	
	public double[] calculateFaceNormals(MemoryAddress face) {
		try (MemorySession session = MemorySession.openConfined()) {
			MemorySegment normal = session.allocateArray(Newton_h.C_DOUBLE, Newton.VEC3D);
			calculateFaceNormals(face, normal);
			return normal.toArray(Newton_h.C_DOUBLE);
		}
	}
	
	public void setFaceMaterial(MemoryAddress face, int materialID) {
		Newton_h.NewtonMeshSetFaceMaterial(address, face, materialID);
	}
}
