package crab.newton.callbacks;// Generated by jextract

import java.lang.foreign.*;
import crab.newton.internal.*;

public interface NewtonGetTimeInMicrosencondsCallback {

    long apply();
    static MemorySegment allocate(NewtonGetTimeInMicrosencondsCallback fi, MemorySession session) {
        return RuntimeHelper.upcallStub(NewtonGetTimeInMicrosencondsCallback.class, fi, constants$3.NewtonGetTimeInMicrosencondsCallback$FUNC, session);
    }
    static NewtonGetTimeInMicrosencondsCallback ofAddress(MemoryAddress addr, MemorySession session) {
        MemorySegment symbol = MemorySegment.ofAddress(addr, 0, session);
        return () -> {
            try {
                return (long)constants$3.NewtonGetTimeInMicrosencondsCallback$MH.invokeExact((Addressable)symbol);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


