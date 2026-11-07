class ToolFailure(str):
    """A tool result that reports a failure rather than a success.

    It is still a str, so the LLM, speak() and print() see the same text;
    my_langgraph checks the type to decide whether terse mode speaks. Building
    a new string from one (concatenation, an f-string) gives a plain str, so
    wrap the final message, not a piece of it.
    """
