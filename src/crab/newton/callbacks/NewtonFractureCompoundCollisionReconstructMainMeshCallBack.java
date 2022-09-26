package crab.newton.callbacks;// Generated by jextract

import java.lang.foreign.*;
import crab.newton.internal.*;

public interface NewtonFractureCompoundCollisionReconstructMainMeshCallBack {

    void apply(java.lang.foreign.MemoryAddress body, java.lang.foreign.MemoryAddress mainMesh, java.lang.foreign.MemoryAddress fracturedCompountCollision);
    static MemorySegment allocate(NewtonFractureCompoundCollisionReconstructMainMeshCallBack fi, MemorySession session) {
        return RuntimeHelper.upcallStub(NewtonFractureCompoundCollisionReconstructMainMeshCallBack.class, fi, constants$12.NewtonFractureCompoundCollisionReconstructMainMeshCallBack$FUNC, session);
    }
    static NewtonFractureCompoundCollisionReconstructMainMeshCallBack ofAddress(MemoryAddress addr, MemorySession session) {
        MemorySegment symbol = MemorySegment.ofAddress(addr, 0, session);
        return (java.lang.foreign.MemoryAddress _body, java.lang.foreign.MemoryAddress _mainMesh, java.lang.foreign.MemoryAddress _fracturedCompountCollision) -> {
            try {
                constants$12.NewtonFractureCompoundCollisionReconstructMainMeshCallBack$MH.invokeExact((Addressable)symbol, (java.lang.foreign.Addressable)_body, (java.lang.foreign.Addressable)_mainMesh, (java.lang.foreign.Addressable)_fracturedCompountCollision);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


