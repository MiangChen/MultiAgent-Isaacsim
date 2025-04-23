# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.

from typing import List
import omni.ui as ui
from isaacsim.gui.components.element_wrappers import (
    Button,
    StringField,
    TextBlock,
    CollapsableFrame,
)
from isaacsim.gui.components.ui_utils import get_style
import asyncio
from .agent import Agent
class UIBuilder:
    def __init__(self):
        self.frames = []
        self.wrapped_ui_elements = []

        self._input_field = None
        self._submit_button = None
        self._output_text_block = None
        self.display_text = ""
        self.agent = Agent()
    def build_ui(self):
        self._create_chat_ui_frame()

    def cleanup(self):
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def on_menu_callback(self):
        pass

    def on_timeline_event(self, event):
        pass

    def on_physics_step(self, step):
        pass

    def on_stage_event(self, event):
        pass

    def _create_chat_ui_frame(self):
        chat_frame = CollapsableFrame("LLM Chat UI", collapsed=False)

        with chat_frame:
            with ui.VStack(style=get_style(), spacing=10, height=0):

                # output
                self._output_text_block = TextBlock(
                    "Outcome",
                    num_lines=45,
                    tooltip="Outcome",
                    include_copy_button=False,
                )

                submit_button = Button(
                    "Send",
                    "Send",
                    tooltip="Click to send",
                    on_click_fn=self._on_submit_clicked,
                )

                self._submit_button = submit_button  
                self.wrapped_ui_elements.append(submit_button) 
                # input
                with ui.HStack(spacing=5, height=120):
                    
                    # text
                    with ui.VStack(width=550):
                        ui.Label("User Input\n", alignment=ui.Alignment.LEFT_TOP)

                        self._input_field = ui.StringField(
                            multiline=True,
                            height=120,
                            style={"background_color": 0x22FFFFFF}
                        )

    def _on_submit_clicked(self):
        asyncio.ensure_future(self._handle_submit_async())

    async def _handle_submit_async(self):

        self._input_field.enabled = False
        if self._submit_button and self._submit_button.button:
            self._submit_button.button.enabled = False

        user_input = self._input_field.model.get_value_as_string()

        self.display_text += f"===========================UserInput===========================\n{user_input}\n\n\n==========================LLMOutcome==========================\n"
        self._output_text_block.set_text(self.display_text+"LLM Thinking...\n\n\n")

        print("User_input: " + user_input)
        result = await self.agent.run(user_input)

        self.display_text += f"{result}\n\n\n"
        self._output_text_block.set_text(self.display_text)

        self._input_field.model.set_value("")
        self._input_field.enabled = True
        if self._submit_button and self._submit_button.button:
            self._submit_button.button.enabled = True

    def add_llm_text(self, text: str):
        self.display_text += f"===========================LLMOutcome===========================\n{text}\n\n\n"
        self._output_text_block.set_text(self.display_text)


