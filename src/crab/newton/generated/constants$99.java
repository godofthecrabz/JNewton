// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$99 {

    static final FunctionDescriptor NewtonMeshCreateNextLayer$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshCreateNextLayer$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshCreateNextLayer",
        constants$99.NewtonMeshCreateNextLayer$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshGetTotalFaceCount$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshGetTotalFaceCount$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetTotalFaceCount",
        constants$99.NewtonMeshGetTotalFaceCount$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshGetTotalIndexCount$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshGetTotalIndexCount$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetTotalIndexCount",
        constants$99.NewtonMeshGetTotalIndexCount$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshGetFaces$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshGetFaces$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetFaces",
        constants$99.NewtonMeshGetFaces$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshGetVertexCount$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshGetVertexCount$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetVertexCount",
        constants$99.NewtonMeshGetVertexCount$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshGetVertexStrideInByte$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshGetVertexStrideInByte$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetVertexStrideInByte",
        constants$99.NewtonMeshGetVertexStrideInByte$FUNC, false
    );
}


