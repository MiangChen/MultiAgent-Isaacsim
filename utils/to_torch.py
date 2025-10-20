# =============================================================================
# To Torch Module - Tensor Conversion Utilities
# =============================================================================
#
# This module provides utility functions for converting various data types
# to PyTorch tensors with specified device, dtype, and gradient requirements.
#
# =============================================================================

# Third-party library imports
import torch


def to_torch(
    data,
    dtype: torch.dtype = torch.float32,
    device: str = None,
    requires_grad: bool = False,
) -> torch.Tensor:
    if data is None:
        return None
    if device is None:
        if hasattr(data, "device"):
            device = data.device
        else:
            device = "cpu"

    if isinstance(data, torch.Tensor):
        return data.to(device=device, dtype=dtype)
    return torch.as_tensor(data, dtype=dtype, device=device).requires_grad_(
        requires_grad
    )
