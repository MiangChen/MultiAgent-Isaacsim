import sounddevice as sd
import numpy as np
from scipy.io.wavfile import write
from faster_whisper import WhisperModel
from openai import OpenAI
import re, os
import pandas as pd
import pyttsx3
import openai
from files.assets_scripts_linux import PATH_PROJECT  # 这里用了项目的路径， 如果找不到某个文件，可能是这个没设置
openai.api_key = "sk-MQ8Zb5grxtM4hniFfyAWeblLcDxwNKG9tP64PGrfAaAePbr7"
openai.api_base = "https://vip.dmxapi.com/L"

# 初始化 TTS
engine = pyttsx3.init()
engine.setProperty('voice', 'com.apple.speech.synthesis.voice.ting-ting')  # mac 上的中文语音

# 初始化 Whisper 模型（可以改成 "medium"、"large-v2"）
model = WhisperModel("small", device="cpu", compute_type="int8")

# 候选位置
candidate_locations = ['厨房', '卧室', '客厅', '卫生间', '阳台', "咖啡机"]

# 文件的根路径
def record_audio(filename="recording.wav", duration=5, fs=16000):
    print("🎤 开始录音，说话吧...")
    recording = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='int16')
    sd.wait()
    write(filename, fs, recording)
    print("✅ 录音完成")

def transcribe_audio(path):
    segments, info = model.transcribe(path, beam_size=5)
    text = "".join([seg.text for seg in segments])
    return text.strip()

def get_xyz_from_excel(location_name, excel_path=f'{PATH_PROJECT}/llm/home.xlsx'):
    df = pd.read_excel(excel_path, header=None)  # 👈 不使用第一行作为列名
    row = df[df.iloc[:, 0] == location_name]     # 第一列是位置名
    if not row.empty:
        x = row.iloc[0, 1]  # 第二列 X
        y = row.iloc[0, 2]  # 第三列 Y
        z = row.iloc[0, 3]  # 第四列 Z
        return x, y, z
    else:
        return None, None, None

def extract_location(text, candidates):
    prompt = f"""你是一个家居助理。我会给你一句中文命令，请你从以下候选位置中判断这个命令最可能是在哪个位置执行的：
候选位置：{", ".join(candidates)}
命令：{text}
请你在[]中只返回一个位置，不要解释。"""
    # response = openai.ChatCompletion.create(
    #     model="gpt-3.5-turbo",
    #     messages=[{"role": "user", "content": prompt}]
    # )

    client = OpenAI(api_key="sk-4b2b473f8d8149109dea837a488fa273", base_url="https://api.deepseek.com")

    response = client.chat.completions.create(
        model="deepseek-chat",
        messages=[
            {"role": "system", "content": "你是一个家居助理"},
            {"role": "user", "content": prompt},
        ],
        stream=False
    )
    # import pdb; pdb.set_trace()
    return response.choices[0].message.content

def respond(text, location):
    response = f"我将去{location}完成：{text}"
    print("🤖 回应：", response)
    engine.say(response)
    engine.runAndWait()


def extract_bracket_content(text):
    match = re.search(r'\[([^\[\]]+)\]', text)
    if match:
        return match.group(1)
    return None

def run_once(file_path: str=None):
    if file_path is None:
        record_audio()
        file_path = f"recording.wav"
    text = transcribe_audio(f'{PATH_PROJECT}/llm/{file_path}')
    print("📝 你说的是：", text)
    if not text:
        print("❌ 没听清")
        return
    location = extract_location(text, candidate_locations)
    location = extract_bracket_content(location)
    print(location)
    respond(text, location)
    x, y, z = get_xyz_from_excel(location)
    print(f'x: {x}')
    print(f'y: {y}')
    print(f'z: {z}')
    return x, y, z

# 👉 主程序
if __name__ == "__main__":
    while True:
        input("按下 Enter 开始一次录音（5秒）...")
        x, y, z = run_once()
