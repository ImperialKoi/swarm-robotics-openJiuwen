"""SDK model-client extension for bounded local-model tool arguments.

The installed llama.cpp build can ignore tool_choice. JSON-schema decoding is
reliable, so adapt one native tool's argument schema into a structured response.
The SDK still calls the model and owns all agent/tool execution. This adapter
never chooses a candidate, supplies a vote, or executes a tool itself.
"""

import json
from uuid import uuid4

from jsonschema import validate
from openjiuwen.core.common.clients import get_client_registry
from openjiuwen.core.foundation.llm import (
    AssistantMessageChunk,
    OpenAIModelClient,
    ToolCall,
    UserMessage,
)

PROVIDER = "SwarmMindLocal"


@get_client_registry().register_client(PROVIDER, "llm")
class LocalToolArguments(OpenAIModelClient):
    async def invoke(self, messages, *, tools=None, **kwargs):
        if len(tools or []) != 1:
            raise ValueError("local native client requires exactly one authorized stage tool")
        tool = tools[0]
        name, schema = tool.name, tool.parameters
        prompt = UserMessage(content=f"Call {name}. Return only its JSON arguments matching this schema: "
                             + json.dumps(schema, separators=(",", ":")))
        # tools and response_format cannot both constrain llama.cpp decoding.
        # Reuse the SDK's HTTP client, accounting, error handling and timeouts.
        self.model_config.tool_choice = "none"
        result = await super().invoke(
            [*messages, prompt], tools=None,
            response_format={"type": "json_schema", "json_schema": {
                "name": name, "strict": True, "schema": schema}}, **kwargs)
        arguments = json.loads(result.content)
        validate(arguments, schema)  # fail closed even if the endpoint ignores its schema
        return result.model_copy(update={
            "content": "", "finish_reason": "tool_calls",
            "tool_calls": [ToolCall(id=f"rescue-{uuid4().hex}", type="function", name=name,
                                    arguments=json.dumps(arguments), index=0)],
        })

    async def stream(self, messages, *, tools=None, **kwargs):
        result = await self.invoke(messages, tools=tools, **kwargs)
        yield AssistantMessageChunk(**result.model_dump())
