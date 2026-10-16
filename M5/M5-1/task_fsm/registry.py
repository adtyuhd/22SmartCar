from .nodes import (
    WaitForTrigger,
    NavigateTo,
    DetectTarget,
    ExecuteGrasp,
    SpeakTTS
)


NODE_TYPES = {
    "WaitForTrigger": WaitForTrigger,
    "NavigateTo": NavigateTo,
    "DetectTarget": DetectTarget,
    "ExecuteGrasp": ExecuteGrasp,
    "SpeakTTS": SpeakTTS
}


def create_node(name, node_type, params, ctx):
    node_class = NODE_TYPES.get(node_type)

    if node_class is None:
        available = ", ".join(NODE_TYPES)
        raise ValueError(
            f"Unknown node type '{node_type}'. "
            f"Available types: {available}"
        )

    return node_class(name, params, ctx)