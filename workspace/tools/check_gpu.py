#!/usr/bin/env python3
import torch
import jax

torch_ok = torch.cuda.is_available()
print(f"torch: cuda_available={torch_ok}", 
      f"device={torch.cuda.get_device_name(0)}" if torch_ok else "")

jax_devices = jax.devices()
jax_ok = any(d.platform == "gpu" for d in jax_devices)
print(f"jax: devices={jax_devices}, gpu_ok={jax_ok}")

if not (torch_ok and jax_ok):
    raise SystemExit("GPU not available for torch and/or jax")
