from typing import Dict, Any


def form_feedback(
    self, status: str = "processing", reason: str = "none", progress: int = 100
) -> Dict[str, Any]:
    return dict(
        status=status,
        reason=reason,
        progress=progress,
    )
