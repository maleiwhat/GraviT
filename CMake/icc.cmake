SET(GVT_ARCH_FLAGS__SSE3  "-xsse3")
SET(GVT_ARCH_FLAGS__SSSE3 "-xssse3")
SET(GVT_ARCH_FLAGS__SSE41 "-xsse4.1")
SET(GVT_ARCH_FLAGS__SSE42 "-xsse4.2")
SET(GVT_ARCH_FLAGS__SSE   "-xsse4.2")
SET(GVT_ARCH_FLAGS__AVX   "-xAVX")
SET(GVT_ARCH_FLAGS__AVX2  "-xCORE-AVX2")

SET(CMAKE_CXX_COMPILER "icpc")
SET(CMAKE_C_COMPILER "icc")
SET(CMAKE_CXX_FLAGS "-Wall -fPIC -static-intel")
SET(CMAKE_CXX_FLAGS_DEBUG          "-g -O3 ")
SET(CMAKE_CXX_FLAGS_RELEASE        "-DNDEBUG    -O3 -no-ansi-alias -restrict -fp-model fast -fimf-precision=low -no-prec-div -no-prec-sqrt")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-DNDEBUG -g -O3 -no-ansi-alias -restrict -fp-model fast -fimf-precision=low -no-prec-div -no-prec-sqrt")
SET(CMAKE_EXE_LINKER_FLAGS "") 
