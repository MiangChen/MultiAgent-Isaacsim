"""
Copyright (c) 2024 WindyLab of Westlake University, China
All rights reserved.

This software is provided "as is" without warranty of any kind, either
express or implied, including but not limited to the warranties of
merchantability, fitness for a particular purpose, or non-infringement.
In no event shall the authors or copyright holders be liable for any
claim, damages, or other liability, whether in an action of contract,
tort, or otherwise, arising from, out of, or in connection with the
software or the use or other dealings in the software.
"""

from abc import ABC, abstractmethod


class BaseLLM(ABC):
    """
    Base class for interacting with different LLM models.

    This class defines the basic methods and properties required for interacting
    with various LLM models from different manufacturers.

    Args:
        model (str): Model to use.
        memorize (bool): Whether to memorize conversation history.
        stream_output (bool): Whether to use streaming output.
    """

    def __init__(
        self, model: str, memorize: bool = False, stream_output: bool = False
    ) -> None:
        self._model = model
        self._memorize = memorize
        self._stream_output = stream_output
        self._memories = []  # Current memories
        self._response: str
        self.system_prompt = "You are a helpful assistant."

    def reset(self, system_prompt: str) -> None:
        if system_prompt:
            self.system_prompt = system_prompt
        self._memories = []
        # self._memories.append({"role": "system", "content": self.system_prompt})

    async def ask(self, prompt: str | list, temperature=1) -> str:
        """
        Asynchronously generate an answer from the model based on the given prompt.
        """
        self._memories.append({"role": "user", "content": prompt})
        response = await self._ask_with_retry(temperature)
        if self._memorize:
            self._memories.append({"role": "assistant", "content": response})
        return response

    @abstractmethod
    async def _ask_with_retry(self, temperature: float) -> str:
        """
        Abstract method to be implemented by subclasses for the actual call to the model with retry logic.
        """

        raise NotImplementedError