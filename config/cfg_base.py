# =============================================================================
# Config Base Module - Base Configuration Class
# =============================================================================
#
# This module provides the base configuration class using Pydantic for
# type validation and configuration management across the application.
#
# =============================================================================

# Third-party library imports
from pydantic import BaseModel


class CfgBase(BaseModel):
    def update(self, **kwargs):
        return self.model_copy(update=kwargs, deep=True)
