// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public interface NewtonPostUpdateCallback {

    void apply(jdk.incubator.foreign.MemoryAddress x0, float x1);
    static NativeSymbol allocate(NewtonPostUpdateCallback fi, ResourceScope scope) {
        return RuntimeHelper.upcallStub(NewtonPostUpdateCallback.class, fi, constants$1.NewtonPostUpdateCallback$FUNC, "(Ljdk/incubator/foreign/MemoryAddress;F)V", scope);
    }
    static NewtonPostUpdateCallback ofAddress(MemoryAddress addr, ResourceScope scope) {
        NativeSymbol symbol = NativeSymbol.ofAddress("NewtonPostUpdateCallback::" + Long.toHexString(addr.toRawLongValue()), addr, scope);
return (jdk.incubator.foreign.MemoryAddress x0, float x1) -> {
            try {
                constants$1.NewtonPostUpdateCallback$MH.invokeExact(symbol, (jdk.incubator.foreign.Addressable)x0, x1);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}

