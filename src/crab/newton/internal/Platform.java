package crab.newton.internal;

public enum Platform {
    LINUX("linux", ".so"),
    MACOS("macos", ".dylib"),
    WINDOWS("windows", ".dll");

    public enum Architecture {
        X64(true),
        X86(false),
        ARM64(true),
        ARM32(false);

        static final Architecture current;

        final boolean is64Bit;

        static {
            String  osArch  = System.getProperty("os.arch");
            boolean is64Bit = osArch.contains("64") || osArch.startsWith("armv8");

            current = osArch.startsWith("arm") || osArch.startsWith("aarch64")
                    ? (is64Bit ? Architecture.ARM64 : Architecture.ARM32)
                    : (is64Bit ? Architecture.X64 : Architecture.X86);
        }

        Architecture(boolean is64Bit) {
            this.is64Bit = is64Bit;
        }
    }

    private static final Platform current;
    private static final String newtonPath;

    static {
        String osName = System.getProperty("os.name");
        if (osName.startsWith("Windows")) {
            current = WINDOWS;
        } else if (osName.startsWith("Linux") || osName.startsWith("FreeBSD") || osName.startsWith("SunOS") || osName.startsWith("Unix")) {
            current = LINUX;
        } else if (osName.startsWith("Mac OS X") || osName.startsWith("Darwin")) {
            current = MACOS;
        } else {
            throw new LinkageError("Unknown platform: " + osName);
        }

        newtonPath = switch (current) {
            case LINUX, MACOS -> "unimplemented";
            case WINDOWS -> current.name + "_" + Architecture.current.name().toLowerCase() + "/crab/jnewton/newton.dll";
        };
    }

    public final String name;
    public final String fileType;

    Platform(String name, String fileType) {
        this.name = name;
        this.fileType = fileType;
    }

    public String getName() {
        return name;
    }

    public String getFileType() {
        return fileType;
    }

    public static Platform get() {
        return current;
    }

    public static String getNewtonPath() {
        return newtonPath;
    }

    public static Architecture getArchitecture() {
        return Architecture.current;
    }
}
