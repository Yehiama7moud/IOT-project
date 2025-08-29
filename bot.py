from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse, HTMLResponse
import paho.mqtt.client as mqtt
from langchain.agents import initialize_agent, AgentType, Tool
from langchain_google_genai import ChatGoogleGenerativeAI
import uvicorn

# --- MQTT Setup ---
BROKER = "db214e9cf2184882a22e1639d81e429f.s1.eu.hivemq.cloud"
PORT = 8883
TOPIC_GATE = "trafficApp/gate"
TOPIC_LIGHT = "trafficApp/light"
TOPIC_TRAFFIC = "trafficApp/traffic"

mqtt_client = mqtt.Client()


def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT with code", rc)
    client.subscribe("trafficApp/#")  # subscribe to all traffic topics


def on_message(client, userdata, msg):
    print(f"[MQTT] {msg.topic} => {msg.payload.decode()}")


mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(BROKER, PORT, 60)
mqtt_client.loop_start()


# --- Tool Functions ---
def open_gate(_=None):
    mqtt_client.publish(TOPIC_GATE, "OPEN")
    return "Gate opened"


def close_gate(_=None):
    mqtt_client.publish(TOPIC_GATE, "CLOSE")
    return "Gate closed"


def set_traffic_light(color: str):
    color = color.upper()
    if color not in ["RED", "YELLOW", "GREEN"]:
        return "Invalid color. Use RED, YELLOW, or GREEN."
    mqtt_client.publish(TOPIC_TRAFFIC, color)
    return f"Traffic light set to {color}"


def set_mode(mode: str):
    mode = mode.upper()
    if mode not in ["DAY", "NIGHT"]:
        return "Invalid mode. Use DAY or NIGHT."
    mqtt_client.publish(TOPIC_LIGHT, mode)
    return f"Mode set to {mode}"


# --- LangChain Tools ---
tools = [
    Tool(name="Open Gate", func=open_gate, description="Opens the gate barrier."),
    Tool(name="Close Gate", func=close_gate, description="Closes the gate barrier."),
    Tool(name="Set Traffic Light", func=set_traffic_light,
         description="Set traffic light to RED, YELLOW, or GREEN."),
    Tool(name="Set Mode", func=set_mode, description="Set mode to DAY or NIGHT."),
]

system_prompt = """
You are a Traffic Assistant Chatbot.
You control a smart traffic light and gate system.

Available tools:
- Open Gate
- Close Gate
- Set Traffic Light (RED, YELLOW, GREEN)
- Set Mode (DAY, NIGHT)

Rules:
1. ALWAYS use the tools — don’t just explain.
2. Only valid commands are accepted.
3. If invalid, reply with: "I cannot do that."
"""

llm = ChatGoogleGenerativeAI(
    model="gemini-2.5-flash",
    google_api_key="AIzaSyBRQIH-V_VOmvplsjRtIgCKVUewrMAEruM"
)

agent = initialize_agent(
    tools=tools,
    llm=llm,
    agent=AgentType.STRUCTURED_CHAT_ZERO_SHOT_REACT_DESCRIPTION,
    verbose=True,
    handle_parsing_errors=True,
    return_only_outputs=True
)
# --- FastAPI App ---
app = FastAPI()

@app.post("/chat")
async def chat(request: Request):
    data = await request.json()
    user_input = data.get("text", "")
    try:
        reply = agent.invoke({
            "input": user_input,
            "chat_history": [{"role": "system", "content": system_prompt}]
        })
        return JSONResponse({"reply": reply.get("output", "")})
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)

if __name__ == "__main__":
    uvicorn.run("bot:app", host="0.0.0.0", port=8000, reload=True)