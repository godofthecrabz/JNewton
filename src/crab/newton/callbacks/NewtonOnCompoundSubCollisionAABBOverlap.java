package crab.newton.callbacks;// Generated by jextract

import java.lang.foreign.*;
import crab.newton.internal.*;

public interface NewtonOnCompoundSubCollisionAABBOverlap {

    int apply(java.lang.foreign.MemoryAddress contact, float timestep, java.lang.foreign.MemoryAddress body0, java.lang.foreign.MemoryAddress collisionNode0, java.lang.foreign.MemoryAddress body1, java.lang.foreign.MemoryAddress collisionNode1, int threadIndex);
    static MemorySegment allocate(NewtonOnCompoundSubCollisionAABBOverlap fi, MemorySession session) {
        return RuntimeHelper.upcallStub(NewtonOnCompoundSubCollisionAABBOverlap.class, fi, constants$13.NewtonOnCompoundSubCollisionAABBOverlap$FUNC, session);
    }
    static NewtonOnCompoundSubCollisionAABBOverlap ofAddress(MemoryAddress addr, MemorySession session) {
        MemorySegment symbol = MemorySegment.ofAddress(addr, 0, session);
        return (java.lang.foreign.MemoryAddress _contact, float _timestep, java.lang.foreign.MemoryAddress _body0, java.lang.foreign.MemoryAddress _collisionNode0, java.lang.foreign.MemoryAddress _body1, java.lang.foreign.MemoryAddress _collisionNode1, int _threadIndex) -> {
            try {
                return (int)constants$13.NewtonOnCompoundSubCollisionAABBOverlap$MH.invokeExact((Addressable)symbol, (java.lang.foreign.Addressable)_contact, _timestep, (java.lang.foreign.Addressable)_body0, (java.lang.foreign.Addressable)_collisionNode0, (java.lang.foreign.Addressable)_body1, (java.lang.foreign.Addressable)_collisionNode1, _threadIndex);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


