// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public interface NewtonOnAABBOverlap {

    int apply(jdk.incubator.foreign.MemoryAddress x0, float x1, int x2);
    static NativeSymbol allocate(NewtonOnAABBOverlap fi, ResourceScope scope) {
        return RuntimeHelper.upcallStub(NewtonOnAABBOverlap.class, fi, constants$13.NewtonOnAABBOverlap$FUNC, "(Ljdk/incubator/foreign/MemoryAddress;FI)I", scope);
    }
    static NewtonOnAABBOverlap ofAddress(MemoryAddress addr, ResourceScope scope) {
        NativeSymbol symbol = NativeSymbol.ofAddress("NewtonOnAABBOverlap::" + Long.toHexString(addr.toRawLongValue()), addr, scope);
return (jdk.incubator.foreign.MemoryAddress x0, float x1, int x2) -> {
            try {
                return (int)constants$13.NewtonOnAABBOverlap$MH.invokeExact(symbol, (jdk.incubator.foreign.Addressable)x0, x1, x2);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


