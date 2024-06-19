import os
import trace
import sys

# Setup tracing to log to a file with UTF-8 encoding
tracer = trace.Trace(trace=1, count=0)

def load_model():
    import inference
    from inference import get_model
    os.environ["ROBOFLOW_API_KEY"] = "lSddg3rVCpQ9TfAFCo1V"
    model = get_model("kukurus/1")
    return model

# Redirect output to a log file with UTF-8 encoding
sys.stdout = open('trace_output.txt', 'w', encoding='utf-8')

# Run tracing
tracer.run('load_model()')

# Restore standard output
sys.stdout = sys.__stdout__
