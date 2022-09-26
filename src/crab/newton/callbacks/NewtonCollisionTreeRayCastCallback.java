package crab.newton.callbacks;// Generated by jextract

import java.lang.foreign.*;
import crab.newton.internal.*;

public interface NewtonCollisionTreeRayCastCallback {

    float apply(java.lang.foreign.MemoryAddress body, java.lang.foreign.MemoryAddress treeCollision, float intersection, java.lang.foreign.MemoryAddress normal, int faceId, java.lang.foreign.MemoryAddress usedData);
    static MemorySegment allocate(NewtonCollisionTreeRayCastCallback fi, MemorySession session) {
        return RuntimeHelper.upcallStub(NewtonCollisionTreeRayCastCallback.class, fi, constants$8.NewtonCollisionTreeRayCastCallback$FUNC, session);
    }
    static NewtonCollisionTreeRayCastCallback ofAddress(MemoryAddress addr, MemorySession session) {
        MemorySegment symbol = MemorySegment.ofAddress(addr, 0, session);
        return (java.lang.foreign.MemoryAddress _body, java.lang.foreign.MemoryAddress _treeCollision, float _intersection, java.lang.foreign.MemoryAddress _normal, int _faceId, java.lang.foreign.MemoryAddress _usedData) -> {
            try {
                return (float)constants$8.NewtonCollisionTreeRayCastCallback$MH.invokeExact((Addressable)symbol, (java.lang.foreign.Addressable)_body, (java.lang.foreign.Addressable)_treeCollision, _intersection, (java.lang.foreign.Addressable)_normal, _faceId, (java.lang.foreign.Addressable)_usedData);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


