// Generated by jextract

package crab.newton.callbacks;

import crab.newton.internal.*;
import jdk.incubator.foreign.*;
public interface NewtonUserBilateralCallback {

    void apply(jdk.incubator.foreign.MemoryAddress x0, float x1, int x2);
    static NativeSymbol allocate(NewtonUserBilateralCallback fi, ResourceScope scope) {
        return RuntimeHelper.upcallStub(NewtonUserBilateralCallback.class, fi, constants$17.NewtonUserBilateralCallback$FUNC, "(Ljdk/incubator/foreign/MemoryAddress;FI)V", scope);
    }
    static NewtonUserBilateralCallback ofAddress(MemoryAddress addr, ResourceScope scope) {
        NativeSymbol symbol = NativeSymbol.ofAddress("NewtonUserBilateralCallback::" + Long.toHexString(addr.toRawLongValue()), addr, scope);
return (jdk.incubator.foreign.MemoryAddress x0, float x1, int x2) -> {
            try {
                constants$17.NewtonUserBilateralCallback$MH.invokeExact(symbol, (jdk.incubator.foreign.Addressable)x0, x1, x2);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


