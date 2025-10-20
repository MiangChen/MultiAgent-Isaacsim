# =============================================================================
# Check PDDL Engine Module - PDDL Planning Engine Verification
# =============================================================================
#
# This module provides verification and testing functionality for PDDL
# (Planning Domain Definition Language) engines and planning capabilities.
#
# =============================================================================

# Third-party library imports
from unified_planning.shortcuts import get_environment
import pyperplan
import up_pyperplan

print(get_environment().factory.engines)
