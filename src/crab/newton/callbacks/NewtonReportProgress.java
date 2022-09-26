package crab.newton.callbacks;// Generated by jextract

import java.lang.foreign.*;
import crab.newton.internal.*;

public interface NewtonReportProgress {

    int apply(float normalizedProgressPercent, java.lang.foreign.MemoryAddress userData);
    static MemorySegment allocate(NewtonReportProgress fi, MemorySession session) {
        return RuntimeHelper.upcallStub(NewtonReportProgress.class, fi, constants$18.NewtonReportProgress$FUNC, session);
    }
    static NewtonReportProgress ofAddress(MemoryAddress addr, MemorySession session) {
        MemorySegment symbol = MemorySegment.ofAddress(addr, 0, session);
        return (float _normalizedProgressPercent, java.lang.foreign.MemoryAddress _userData) -> {
            try {
                return (int)constants$18.NewtonReportProgress$MH.invokeExact((Addressable)symbol, _normalizedProgressPercent, (java.lang.foreign.Addressable)_userData);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


