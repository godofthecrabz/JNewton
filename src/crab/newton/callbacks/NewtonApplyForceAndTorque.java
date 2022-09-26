package crab.newton.callbacks;// Generated by jextract

import java.lang.foreign.*;
import crab.newton.internal.*;

public interface NewtonApplyForceAndTorque {

    void apply(java.lang.foreign.MemoryAddress body, float timestep, int threadIndex);
    static MemorySegment allocate(NewtonApplyForceAndTorque fi, MemorySession session) {
        return RuntimeHelper.upcallStub(NewtonApplyForceAndTorque.class, fi, constants$10.NewtonApplyForceAndTorque$FUNC, session);
    }
    static NewtonApplyForceAndTorque ofAddress(MemoryAddress addr, MemorySession session) {
        MemorySegment symbol = MemorySegment.ofAddress(addr, 0, session);
        return (java.lang.foreign.MemoryAddress _body, float _timestep, int _threadIndex) -> {
            try {
                constants$10.NewtonApplyForceAndTorque$MH.invokeExact((Addressable)symbol, (java.lang.foreign.Addressable)_body, _timestep, _threadIndex);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


