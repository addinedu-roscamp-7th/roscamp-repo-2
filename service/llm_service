from fastapi import FastAPI, File, UploadFile
from io import BytesIO
import whisper
import asyncio
import tempfile
import os

app = FastAPI()

model = whisper.load_model("base")

@app.post("/recognize")
async def recognize_audio(file: UploadFile = File(...)):
    try:
        audio_bytes = await file.read()
        audio_stream = BytesIO(audio_bytes)

        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(None, transcribe_audio, audio_stream)
        return {"success": True, "recognized_text": result}

    except Exception as e:
        return {"success": False, "error": str(e)}


def transcribe_audio(audio_stream: BytesIO):
    with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as tmp:
        tmp.write(audio_stream.read())
        tmp_path = tmp.name

    result = model.transcribe(tmp_path)
    os.remove(tmp_path)
    return result["text"]
