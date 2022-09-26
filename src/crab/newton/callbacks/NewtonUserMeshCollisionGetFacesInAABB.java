package crab.newton.callbacks;// Generated by jextract

import java.lang.foreign.*;
import crab.newton.internal.*;

public interface NewtonUserMeshCollisionGetFacesInAABB {

    int apply(java.lang.foreign.MemoryAddress userData, java.lang.foreign.MemoryAddress p0, java.lang.foreign.MemoryAddress p1, java.lang.foreign.MemoryAddress vertexArray, java.lang.foreign.MemoryAddress vertexCount, java.lang.foreign.MemoryAddress vertexStrideInBytes, java.lang.foreign.MemoryAddress indexList, int maxIndexCount, java.lang.foreign.MemoryAddress userDataList);
    static MemorySegment allocate(NewtonUserMeshCollisionGetFacesInAABB fi, MemorySession session) {
        return RuntimeHelper.upcallStub(NewtonUserMeshCollisionGetFacesInAABB.class, fi, constants$7.NewtonUserMeshCollisionGetFacesInAABB$FUNC, session);
    }
    static NewtonUserMeshCollisionGetFacesInAABB ofAddress(MemoryAddress addr, MemorySession session) {
        MemorySegment symbol = MemorySegment.ofAddress(addr, 0, session);
        return (java.lang.foreign.MemoryAddress _userData, java.lang.foreign.MemoryAddress _p0, java.lang.foreign.MemoryAddress _p1, java.lang.foreign.MemoryAddress _vertexArray, java.lang.foreign.MemoryAddress _vertexCount, java.lang.foreign.MemoryAddress _vertexStrideInBytes, java.lang.foreign.MemoryAddress _indexList, int _maxIndexCount, java.lang.foreign.MemoryAddress _userDataList) -> {
            try {
                return (int)constants$7.NewtonUserMeshCollisionGetFacesInAABB$MH.invokeExact((Addressable)symbol, (java.lang.foreign.Addressable)_userData, (java.lang.foreign.Addressable)_p0, (java.lang.foreign.Addressable)_p1, (java.lang.foreign.Addressable)_vertexArray, (java.lang.foreign.Addressable)_vertexCount, (java.lang.foreign.Addressable)_vertexStrideInBytes, (java.lang.foreign.Addressable)_indexList, _maxIndexCount, (java.lang.foreign.Addressable)_userDataList);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


