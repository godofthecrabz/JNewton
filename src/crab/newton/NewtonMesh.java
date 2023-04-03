package crab.newton;

import crab.newton.callbacks.NewtonDeserializeCallback;
import crab.newton.callbacks.NewtonReportProgress;
import crab.newton.callbacks.NewtonSerializeCallback;
import crab.newton.internal.*;
import java.lang.foreign.*;

public class NewtonMesh {
	
	protected final MemorySegment address;
	
	protected NewtonMesh(MemorySegment address) {
		this.address = address;
	}
	
	/**
	 * Creates a new {@code NewtonMesh}
	 * @param world - {@code NewtonWorld} to allocate NewtonMesh
	 * @return new {@code NewtonMesh}
	 */
	public static NewtonMesh create(NewtonWorld world) {
		return new NewtonMesh(Newton.NewtonMeshCreate(world.address));
	}
	
	/**
	 * Create a new {@code NewtonMesh} from an existing mesh
	 * @param mesh - {@code NewtonMesh} to be copied
	 * @return new {@code NewtonMesh}
	 */
	public NewtonMesh createFromMesh() {
		return new NewtonMesh(Newton.NewtonMeshCreateFromMesh(address));
	}
	
	public NewtonMesh createTetrahedraIsoSurface() {
		return new NewtonMesh(Newton.NewtonMeshCreateTetrahedraIsoSurface(address));
	}
	
	public static void clearVertexFormat(MemorySegment meshVertexFormat) {
		Newton.NewtonMeshClearVertexFormat(meshVertexFormat);
	}
	
	public void destroy() {
		Newton.NewtonMeshDestroy(address);
	}

	public void serialize(MemorySegment serializeFunction, MemorySegment serializeHandle) {
		Newton.NewtonMeshSerialize(address, serializeFunction, serializeHandle);
	}
	
	public void serialize(NewtonSerializeCallback serializeFunction, MemorySegment serializeHandle, SegmentScope scope) {
		MemorySegment serializeFunc = NewtonSerializeCallback.allocate(serializeFunction, scope);
		Newton.NewtonMeshSerialize(address, serializeFunc, serializeHandle);
	}
	
	public void flipWinding() {
		Newton.NewtonMeshFlipWinding(address);
	}

	public void applyTransform(MemorySegment transform) {
		Newton.NewtonMeshApplyTransform(address, transform);
	}
	
	public void applyTransform(float[] transform) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrix = arena.allocateArray(Newton.C_FLOAT, transform);
			Newton.NewtonMeshApplyTransform(address, matrix);
		}
	}

	public void calculateOOBB(MemorySegment oobb) {
		Newton.NewtonMeshCalculateOOBB(address,
				oobb.asSlice(0L, Newton.MAT4F.byteSize()),
				oobb.asSlice(64L, Newton.C_FLOAT.byteSize()),
				oobb.asSlice(70L, Newton.C_FLOAT.byteSize()),
				oobb.asSlice(74L, Newton.C_FLOAT.byteSize()));
	}
	
	public float[] calculateOOBB() {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment oobb = arena.allocate(Newton.MAT4F_VEC3F);
			Newton.NewtonMeshCalculateOOBB(address, 
					oobb.asSlice(0L, Newton.C_FLOAT.byteSize() * 16), 
					oobb.asSlice(64L, Newton.C_FLOAT.byteSize()), 
					oobb.asSlice(70L, Newton.C_FLOAT.byteSize()), 
					oobb.asSlice(74L, Newton.C_FLOAT.byteSize()));
			return oobb.toArray(Newton.C_FLOAT);
		}
	}
	
	public void calculateVertexNormals(float angleInRadians) {
		Newton.NewtonMeshCalculateVertexNormals(address, angleInRadians);
	}

	public void applySphericalMapping(int material, MemorySegment alignMatrix) {
		Newton.NewtonMeshApplySphericalMapping(address, material, alignMatrix);
	}
	
	public void applySphericalMapping(int material, float[] alignMatrix) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrix = arena.allocateArray(Newton.C_FLOAT, alignMatrix);
			Newton.NewtonMeshApplySphericalMapping(address, material, matrix);
		}
	}

	public void applyCylindricalMapping(int cylinderMaterial, int capMaterial, MemorySegment alignMatrix) {
		Newton.NewtonMeshApplyCylindricalMapping(address, cylinderMaterial, capMaterial, alignMatrix);
	}
	
	public void applyCylindricalMapping(int cylinderMaterial, int capMaterial, float[] alignMatrix) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrix = arena.allocateArray(Newton.C_FLOAT, alignMatrix);
			Newton.NewtonMeshApplyCylindricalMapping(address, cylinderMaterial, capMaterial, matrix);
		}
	}

	public void applyBoxMapping(int frontMaterial, int sideMaterial, int topMaterial, MemorySegment alignMatrix) {
		Newton.NewtonMeshApplyBoxMapping(address, frontMaterial, sideMaterial, topMaterial, alignMatrix);
	}
	
	public void applyBoxMapping(int frontMaterial, int sideMaterial, int topMaterial, float[] alignMatrix) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment matrix = arena.allocateArray(Newton.C_FLOAT, alignMatrix);
			Newton.NewtonMeshApplyBoxMapping(address, frontMaterial, sideMaterial, topMaterial, matrix);
		}
	}

	public void applyAngleBasedMapping(int material, MemorySegment reportProgressCallback, MemorySegment reportProgressUserData, MemorySegment alignMatrix) {
		Newton.NewtonMeshApplyAngleBasedMapping(address, material, reportProgressCallback, reportProgressUserData, alignMatrix);
	}

	public void applyAngleBasedMapping(int material, NewtonReportProgress reportProgressCallback, MemorySegment reportProgressUserData, MemorySegment alignMatrix, SegmentScope scope) {
		MemorySegment callback = NewtonReportProgress.allocate(reportProgressCallback, scope);
		Newton.NewtonMeshApplyAngleBasedMapping(address, material, callback, reportProgressUserData, alignMatrix);
	}
	
	public void applyAngleBasedMapping(int material, NewtonReportProgress reportProgressCallback, MemorySegment reportProgressUserData, float[] alignMatrix, SegmentScope scope) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment callback = NewtonReportProgress.allocate(reportProgressCallback, scope);
			MemorySegment matrix = arena.allocateArray(Newton.C_FLOAT, alignMatrix);
			Newton.NewtonMeshApplyAngleBasedMapping(address, material, callback, reportProgressUserData, matrix);
		}
	}
	
	public void createTetrahedraLinearBlendSkinWeightsChannel(NewtonMesh skinMesh) {
		Newton.NewtonCreateTetrahedraLinearBlendSkinWeightsChannel(address, skinMesh.address);
	}
	
	public void optimize() {
		Newton.NewtonMeshOptimize(address);
	}
	
	public void optimizePoints() {
		Newton.NewtonMeshOptimizePoints(address);
	}
	
	public void optimizeVertex() {
		Newton.NewtonMeshOptimizeVertex(address);
	}
	
	public boolean isOpenMesh() {
		return Newton.NewtonMeshIsOpenMesh(address) == 1;
	}
	
	public void fixTJoints() {
		Newton.NewtonMeshFixTJoints(address);
	}
	
	public void polygonize() {
		Newton.NewtonMeshPolygonize(address);
	}
	
	public void triangulate() {
		Newton.NewtonMeshTriangulate(address);
	}
	
	public NewtonMesh convexMeshIntersection(NewtonMesh convexMesh) {
		return new NewtonMesh(Newton.NewtonMeshConvexMeshIntersection(address, convexMesh.address));
	}
	
	public void beginBuild() {
		Newton.NewtonMeshBeginBuild(address);
	}
	
	public void beginFace() {
		Newton.NewtonMeshBeginFace(address);
	}
	
	public void addPoint(double x, double y, double z) {
		Newton.NewtonMeshAddPoint(address, x, y, z);
	}
	
	public void addLayer(int layerIndex) {
		Newton.NewtonMeshAddLayer(address, layerIndex);
	}
	
	public void addMaterial(int materialIndex) {
		Newton.NewtonMeshAddMaterial(address, materialIndex);
	}
	
	public void addNormal(float x, float y, float z) {
		Newton.NewtonMeshAddNormal(address, x, y, z);
	}
	
	public void addBiNormal(float x, float y, float z) {
		Newton.NewtonMeshAddBinormal(address, x, y, z);
	}
	
	public void addUV0(float u, float v) {
		Newton.NewtonMeshAddUV0(address, u, v);
	}
	
	public void addUV1(float u, float v) {
		Newton.NewtonMeshAddUV1(address, u, v);
	}
	
	public void addVertexColor(float r, float g, float b, float a) {
		Newton.NewtonMeshAddVertexColor(address, r, g, b, a);
	}
	
	public void endFace() {
		Newton.NewtonMeshEndFace(address);
	}
	
	public void endBuild() {
		Newton.NewtonMeshEndBuild(address);
	}
	
	public void buildFromVertexListIndexList(MemorySegment meshVertexFormat) {
		Newton.NewtonMeshBuildFromVertexListIndexList(address, meshVertexFormat);
	}
	
	public int getPointCount() {
		return Newton.NewtonMeshGetPointCount(address);
	}
	
	public MemorySegment getIndexToVertexMap() {
		return Newton.NewtonMeshGetIndexToVertexMap(address);
	}
	
	//public int[] getIndexToVertexMap(int count) {
	//	MemorySegment indexPtr = getIndexToVertexMap();
	//	return MemorySegment.ofAddress(indexPtr.address(), Newton.C_INT.byteSize() * count, SegmentScope.global()).toArray(Newton.C_INT);
	//}
	
	public void getVertexDoubleChannel(int vertexStrideInBytes, MemorySegment vertexBuffer) {
		Newton.NewtonMeshGetVertexDoubleChannel(address, vertexStrideInBytes, vertexBuffer);
	}
	
	public double[] getVertexDoubleChannel(long vertexCount) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment vertexBuffer = arena.allocateArray(Newton.C_DOUBLE, vertexCount * 3);
			getVertexDoubleChannel((int) (Newton.C_DOUBLE.byteSize() * 3), vertexBuffer);
			return vertexBuffer.toArray(Newton.C_DOUBLE);
		}
	}
	
	public void getVertexChannel(int vertexStrideInBytes, MemorySegment vertexBuffer) {
		Newton.NewtonMeshGetVertexChannel(address, vertexStrideInBytes, vertexBuffer);
	}
	
	public float[] getVertexChannel(long vertexCount) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment vertexBuffer = arena.allocateArray(Newton.C_FLOAT, vertexCount * 3);
			getVertexChannel((int) (Newton.C_FLOAT.byteSize() * 3), vertexBuffer);
			return vertexBuffer.toArray(Newton.C_FLOAT);
		}
	}
	
	public void getNormalChannel(int vertexStrideInBytes, MemorySegment normalBuffer) {
		Newton.NewtonMeshGetNormalChannel(address, vertexStrideInBytes, normalBuffer);
	}
	
	public float[] getNormalChannel(long vertexCount) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment normalBuffer = arena.allocateArray(Newton.C_FLOAT, vertexCount * 3);
			getNormalChannel((int) (Newton.C_FLOAT.byteSize() * 3), normalBuffer);
			return normalBuffer.toArray(Newton.C_FLOAT);
		}
	}
	
	public void getBiNormalChannel(int vertexStrideInBytes, MemorySegment biNormalBuffer) {
		Newton.NewtonMeshGetBinormalChannel(address, vertexStrideInBytes, biNormalBuffer);
	}
	
	public float[] getBiNormalChannel(long vertexCount) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment biNormalBuffer = arena.allocateArray(Newton.C_FLOAT, vertexCount * 3);
			getBiNormalChannel((int) (Newton.C_FLOAT.byteSize() * 3), biNormalBuffer);
			return biNormalBuffer.toArray(Newton.C_FLOAT);
		}
	}
	
	public void getUV0Channel(int vertexStrideInBytes, MemorySegment uvBuffer) {
		Newton.NewtonMeshGetUV0Channel(address, vertexStrideInBytes, uvBuffer);
	}
	
	public float[] getUV0Channel(long vertexCount) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment uvBuffer = arena.allocateArray(Newton.C_FLOAT, vertexCount * 2);
			getUV0Channel((int) (Newton.C_FLOAT.byteSize() * 2), uvBuffer);
			return uvBuffer.toArray(Newton.C_FLOAT);
		}
	}
	
	public void getUV1Channel(int vertexStrideInBytes, MemorySegment uvBuffer) {
		Newton.NewtonMeshGetUV1Channel(address, vertexStrideInBytes, uvBuffer);
	}
	
	public float[] getUV1Channel(long vertexCount) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment uvBuffer = arena.allocateArray(Newton.C_FLOAT, vertexCount * 2);
			getUV1Channel((int) (Newton.C_FLOAT.byteSize() * 2), uvBuffer);
			return uvBuffer.toArray(Newton.C_FLOAT);
		}
	}
	
	public void getVertexColorChannel(int vertexStrideInBytes, MemorySegment colorBuffer) {
		Newton.NewtonMeshGetVertexColorChannel(address, vertexStrideInBytes, colorBuffer);
	}
	
	public float[] getVertexColorChannel(long vertexCount) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment colorBuffer = arena.allocateArray(Newton.C_FLOAT, vertexCount * 4);
			getVertexColorChannel((int) (Newton.C_FLOAT.byteSize() * 4), colorBuffer);
			return colorBuffer.toArray(Newton.C_FLOAT);
		}
	}
	
	public boolean hasNormalChannel() {
		return Newton.NewtonMeshHasNormalChannel(address) == 1;
	}
	
	public boolean hasBiNormalChannel() {
		return Newton.NewtonMeshHasBinormalChannel(address) == 1;
	}
	
	public boolean hasUV0Channel() {
		return Newton.NewtonMeshHasUV0Channel(address) == 1;
	}
	
	public boolean hasUV1Channel() {
		return Newton.NewtonMeshHasUV1Channel(address) == 1;
	}
	
	public boolean hasVertexColorChannel() {
		return Newton.NewtonMeshHasVertexColorChannel(address) == 1;
	}
	
	public MemorySegment beginHandle() {
		return Newton.NewtonMeshBeginHandle(address);
	}
	
	public void endHandle(MemorySegment handle) {
		Newton.NewtonMeshEndHandle(address, handle);
	}
	
	public int firstMaterial(MemorySegment handle) {
		return Newton.NewtonMeshFirstMaterial(address, handle);
	}
	
	public int nextMaterial(MemorySegment handle, int materialID) {
		return Newton.NewtonMeshNextMaterial(address, handle, materialID);
	}
	
	public int materialGetMaterial(MemorySegment handle, int materialID) {
		return Newton.NewtonMeshMaterialGetMaterial(address, handle, materialID);
	}
	
	public int materialGetIndexCount(MemorySegment handle, int materialID) {
		return Newton.NewtonMeshMaterialGetIndexCount(address, handle, materialID);
	}
	
	public void materialGetIndexStream(MemorySegment handle, int materialID, MemorySegment indexBuffer) {
		Newton.NewtonMeshMaterialGetIndexStream(address, handle, materialID, indexBuffer);
	}
	
	public int[] materialGetIndexStream(MemorySegment handle, int materialID, long indexCount) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment indexStream = arena.allocateArray(Newton.C_INT, indexCount);
			materialGetIndexStream(handle, materialID, indexStream);
			return indexStream.toArray(Newton.C_INT);
		}
	}
	
	public void materialGetIndexStreamShort(MemorySegment handle, int materialID, MemorySegment indexBuffer) {
		Newton.NewtonMeshMaterialGetIndexStreamShort(address, handle, materialID, indexBuffer);
	}
	
	public short[] materialGetIndexStreamShort(MemorySegment handle, int materialID, long indexCount) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment indexStream = arena.allocateArray(Newton.C_SHORT, indexCount);
			materialGetIndexStreamShort(handle, materialID, indexStream);
			return indexStream.toArray(Newton.C_SHORT);
		}
	}
	
	public MemorySegment createFirstSingleSegment() {
		return Newton.NewtonMeshCreateFirstSingleSegment(address);
	}
	
	public MemorySegment createNextSingleSegment(MemorySegment segment) {
		return Newton.NewtonMeshCreateNextSingleSegment(address, segment);
	}
	
	public MemorySegment createFirstLayer() {
		return Newton.NewtonMeshCreateFirstLayer(address);
	}
	
	public MemorySegment createNextLayer(MemorySegment segment) {
		return Newton.NewtonMeshCreateNextLayer(address, segment);
	}
	
	public int getTotalFaceCount() {
		return Newton.NewtonMeshGetTotalFaceCount(address);
	}
	
	public int getTotalIndexCount() {
		return Newton.NewtonMeshGetTotalIndexCount(address);
	}
	
	public void getFaces(MemorySegment faceIndexCount, MemorySegment faceMaterial, MemorySegment faceIndices) {
		Newton.NewtonMeshGetFaces(address, faceIndexCount, faceMaterial, faceIndices);
	}
	
	public int getVertexCount() {
		return Newton.NewtonMeshGetVertexCount(address);
	}
	
	public int getVertexStrideInBytes() {
		return Newton.NewtonMeshGetVertexStrideInByte(address);
	}
	
	public MemorySegment getVertexArray() {
		return Newton.NewtonMeshGetVertexArray(address);
	}
	
	//public double[] getVertexArray(int vertexCount, int vertexStride) {
	//	MemorySegment vertexBuffer = getVertexArray();
	//	try (MemorySession session = MemorySession.openConfined()) {
	//		return MemorySegment.ofAddress(vertexBuffer, Newton.C_DOUBLE.byteSize() * (vertexCount * vertexStride), session).toArray(Newton.C_DOUBLE);
	//	}
	//}
	
	public int getVertexBaseCount() {
		return Newton.NewtonMeshGetVertexBaseCount(address);
	}
	
	public void setVertexBaseCount(int baseCount) {
		Newton.NewtonMeshSetVertexBaseCount(address, baseCount);
	}
	
	public MemorySegment getFirstVertex() {
		return Newton.NewtonMeshGetFirstVertex(address);
	}
	
	public MemorySegment getNextVertex(MemorySegment vertex) {
		return Newton.NewtonMeshGetNextVertex(address, vertex);
	}
	
	public int getVertexIndex(MemorySegment vertex) {
		return Newton.NewtonMeshGetVertexIndex(address, vertex);
	}
	
	public MemorySegment getFirstPoint() {
		return Newton.NewtonMeshGetFirstPoint(address);
	}
	
	public MemorySegment getNextPoint(MemorySegment point) {
		return Newton.NewtonMeshGetNextPoint(address, point);
	}
	
	public int getPointIndex(MemorySegment point) {
		return Newton.NewtonMeshGetPointIndex(address, point);
	}
	
	public int getVertexIndexFromPoint(MemorySegment point) {
		return Newton.NewtonMeshGetVertexIndexFromPoint(address, point);
	}
	
	public MemorySegment getFirstEdge() {
		return Newton.NewtonMeshGetFirstEdge(address);
	}
	
	public MemorySegment getNextEdge(MemorySegment edge) {
		return Newton.NewtonMeshGetNextEdge(address, edge);
	}
	
	public void getEdgeIndices(MemorySegment edge, MemorySegment v0, MemorySegment v1) {
		Newton.NewtonMeshGetEdgeIndices(address, edge, v0, v1);
	}
	
	public int[] getEdgeIndices(MemorySegment edge) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment indices = arena.allocateArray(Newton.C_INT, 2);
			getEdgeIndices(edge, indices.asSlice(0L, Newton.C_INT.byteSize()), indices.asSlice(4L, Newton.C_INT.byteSize()));
			return indices.toArray(Newton.C_INT);
		}
	}
	
	public MemorySegment getFirstFace() {
		return Newton.NewtonMeshGetFirstFace(address);
	}
	
	public MemorySegment getNextFace(MemorySegment face) {
		return Newton.NewtonMeshGetNextFace(address, face);
	}
	
	public boolean isFaceOpen(MemorySegment face) {
		return Newton.NewtonMeshIsFaceOpen(address, face) == 1;
	}
	
	public int getFaceMaterial(MemorySegment face) {
		return Newton.NewtonMeshGetFaceMaterial(address, face);
	}
	
	public int getFaceIndexCount(MemorySegment face) {
		return Newton.NewtonMeshGetFaceIndexCount(address, face);
	}
	
	public void getFaceIndices(MemorySegment face, MemorySegment indicesBuffer) {
		Newton.NewtonMeshGetFaceIndices(address, face, indicesBuffer);
	}
	
	public int[] getFaceIndices(MemorySegment face, long indexCount) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment indices = arena.allocateArray(Newton.C_INT, indexCount);
			getFaceIndices(face, indices);
			return indices.toArray(Newton.C_INT);
		}
	}
	
	public void getFacePointIndices(MemorySegment face, MemorySegment indicesBuffer) {
		Newton.NewtonMeshGetFacePointIndices(address, face, indicesBuffer);
	}
	
	public int[] getFacePointIndices(MemorySegment face, long indexCount) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment indices = arena.allocateArray(Newton.C_INT, indexCount);
			getFacePointIndices(face, indices);
			return indices.toArray(Newton.C_INT);
		}
	}
	
	public void calculateFaceNormals(MemorySegment face, MemorySegment normalBuffer) {
		Newton.NewtonMeshCalculateFaceNormal(address, face, normalBuffer);
	}
	
	public double[] calculateFaceNormals(MemorySegment face) {
		try (Arena arena = Arena.openConfined()) {
			MemorySegment normal = arena.allocate(Newton.VEC3D);
			calculateFaceNormals(face, normal);
			return normal.toArray(Newton.C_DOUBLE);
		}
	}
	
	public void setFaceMaterial(MemorySegment face, int materialID) {
		Newton.NewtonMeshSetFaceMaterial(address, face, materialID);
	}
}
