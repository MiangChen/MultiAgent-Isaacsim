import asyncio

from prompt import PROBLEM_TEMPLATE, BASECODE
from gpt import GPT
import re


def parse_text(
        text: str, lang: str = "python", all_matches: bool = False
) -> str | list[str]:
    pattern = rf"```{lang}.*?\s+(.*?)```"
    matches = re.findall(pattern, text, re.DOTALL)

    if not matches:
        # TODO: user-defined error
        error_message = f"Error: No '{lang}' code block found in the text."
        raise ValueError(error_message)

    if all_matches:
        return matches
    else:
        return matches[0]


def instruction2goal(instruction):
    gpt = GPT()
    prompt = PROBLEM_TEMPLATE.format(instruction=instruction, code=BASECODE)
    response = asyncio.run(gpt.ask(prompt))
    print(response)
    return parse_text(response)


if __name__ == '__main__':
    instruction2goal(instruction='I want the robots to transport all items back to the depot.')
