#ifndef HOLOLINK_EMULATION_CUDA_RUNTIME_STUB_H
#define HOLOLINK_EMULATION_CUDA_RUNTIME_STUB_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum cudaError {
    cudaSuccess = 0,
    cudaErrorNotSupported = 801,
} cudaError_t;

typedef enum cudaMemcpyKind {
    cudaMemcpyHostToHost = 0,
    cudaMemcpyHostToDevice = 1,
    cudaMemcpyDeviceToHost = 2,
    cudaMemcpyDeviceToDevice = 3,
    cudaMemcpyDefault = 4,
} cudaMemcpyKind;

static inline cudaError_t cudaMemcpy(void* dst, const void* src, size_t count, cudaMemcpyKind kind) {
    (void)dst;
    (void)src;
    (void)count;
    (void)kind;
    return cudaErrorNotSupported;
}

static inline const char* cudaGetErrorString(cudaError_t error) {
    (void)error;
    return "CUDA support is disabled in this build";
}

#ifdef __cplusplus
}
#endif

#endif
