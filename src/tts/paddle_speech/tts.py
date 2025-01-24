from paddlespeech.cli.tts.infer import TTSExecutor

tts = TTSExecutor()
tts(text="今天天气十分不错。", output="output.wav")
