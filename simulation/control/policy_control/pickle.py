# =============================================================================
# Pickle Module - Custom Pickle Utilities
# =============================================================================
#
# This module provides custom pickle unpickler with class mapping
# for handling legacy serialized objects and normalizers.
#
# =============================================================================

# Standard library imports
import pickle


class Unpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if name == "Normalizer" and module == "rsl_rl.utils.utils":
            from simulation.control.policy_control.normalizer import Normalizer

            return Normalizer
        return super().find_class(module, name)
