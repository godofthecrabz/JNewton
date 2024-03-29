// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * int (*NewtonCreateUserMeshCollision$facesInAABBCallback)(void*,float*,float*,float**,int*,int*,int*,int,int*);
 * }
 */
public interface NewtonCreateUserMeshCollision$facesInAABBCallback {

    int apply(java.lang.foreign.MemorySegment userData, java.lang.foreign.MemorySegment p0, java.lang.foreign.MemorySegment p1, java.lang.foreign.MemorySegment vertexArray, java.lang.foreign.MemorySegment vertexCount, java.lang.foreign.MemorySegment vertexStrideInBytes, java.lang.foreign.MemorySegment indexList, int maxIndexCount, java.lang.foreign.MemorySegment userDataList);
    static MemorySegment allocate(NewtonCreateUserMeshCollision$facesInAABBCallback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$65.NewtonCreateUserMeshCollision$facesInAABBCallback_UP$MH, fi, constants$65.NewtonCreateUserMeshCollision$facesInAABBCallback$FUNC, scope);
    }
    static NewtonCreateUserMeshCollision$facesInAABBCallback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _userData, java.lang.foreign.MemorySegment _p0, java.lang.foreign.MemorySegment _p1, java.lang.foreign.MemorySegment _vertexArray, java.lang.foreign.MemorySegment _vertexCount, java.lang.foreign.MemorySegment _vertexStrideInBytes, java.lang.foreign.MemorySegment _indexList, int _maxIndexCount, java.lang.foreign.MemorySegment _userDataList) -> {
            try {
                return (int)constants$66.NewtonCreateUserMeshCollision$facesInAABBCallback_DOWN$MH.invokeExact(symbol, _userData, _p0, _p1, _vertexArray, _vertexCount, _vertexStrideInBytes, _indexList, _maxIndexCount, _userDataList);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


