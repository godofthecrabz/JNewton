package crab.newton.callbacks;// Generated by jextract

import java.lang.foreign.*;
import crab.newton.internal.*;

public interface NewtonUserMeshCollisionCollideCallback {

    void apply(java.lang.foreign.MemoryAddress collideDescData, java.lang.foreign.MemoryAddress continueCollisionHandle);
    static MemorySegment allocate(NewtonUserMeshCollisionCollideCallback fi, MemorySession session) {
        return RuntimeHelper.upcallStub(NewtonUserMeshCollisionCollideCallback.class, fi, constants$7.NewtonUserMeshCollisionCollideCallback$FUNC, session);
    }
    static NewtonUserMeshCollisionCollideCallback ofAddress(MemoryAddress addr, MemorySession session) {
        MemorySegment symbol = MemorySegment.ofAddress(addr, 0, session);
        return (java.lang.foreign.MemoryAddress _collideDescData, java.lang.foreign.MemoryAddress _continueCollisionHandle) -> {
            try {
                constants$7.NewtonUserMeshCollisionCollideCallback$MH.invokeExact((Addressable)symbol, (java.lang.foreign.Addressable)_collideDescData, (java.lang.foreign.Addressable)_continueCollisionHandle);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


