
from langchain.memory import ConversationBufferMemory
from langchain_openai import ChatOpenAI
from langchain.agents import AgentExecutor, Tool, create_openai_functions_agent
from langchain.prompts import ChatPromptTemplate, MessagesPlaceholder
from .tools import ToolFunctions
from .getprim import SceneInfo
from .model_manager import model_manager


class Agent:
    def __init__(self):

        api_base, api_key, model = model_manager.allocate(model_family="QWEN")

        self.llm = ChatOpenAI(
            temperature=0,
            openai_api_key=api_key,
            model=model,
            base_url=api_base
        )

        self.tool_impl = ToolFunctions()
        self.tools = self.tool_impl.tools

        self.memory = ConversationBufferMemory(
            memory_key="chat_history",
            input_key = "input",
            return_messages=True
        )

        self.scene = SceneInfo()
        scene_info = self.scene.get_prim_info()

        self.prompt = ChatPromptTemplate.from_messages([
            ("system",
             """Now you are an agent in IsaacSim -- a 3D simulation software for robotics.
Your task is to use various tools to deal with the request of the user.
If using only a single tool cannot solve the problem, you should use multiple tools in a certain order.
If the user's request is vague, you should ask follow-up questions to clarify the parameters.
Only after the user gives complete information should you start using tools.
You are not allowed to use any other tools except the ones provided.
Strictly follow the format of example to call a tool. Correct format is very important!

This is the current prim data of the whole stage:
{scene_info}
"""),
            MessagesPlaceholder(variable_name="chat_history"),
            ("human", "{input}"),
            MessagesPlaceholder(variable_name="agent_scratchpad"),
        ])

        agent = create_openai_functions_agent(
            llm=self.llm,
            tools=self.tools,
            prompt=self.prompt
        )

        self.agent = AgentExecutor(
            agent=agent,
            tools=self.tools,
            memory=self.memory,
            verbose=True,
            handle_parsing_errors=True
        )

        print("Agent Created!")

    async def run(self, user_input: str) -> str:
        scene_info = self.scene.get_prim_info()

        print(scene_info)

        result = await self.agent.ainvoke({"input": user_input,
                                           "scene_info": scene_info})

        for i, (tool_input, tool_output) in enumerate(result.get("intermediate_steps", [])):
            print(f"\n--- Step {i + 1} ---")
            print("Tool Used:", tool_input.tool)
            print("Tool Input:", tool_input.tool_input)
            print("Tool Output:", tool_output)

        return result["output"]


