from paddlespeech.cli.asr.infer import ASRExecutor

asr = ASRExecutor()
result = asr(audio_file="yutian.wav")
print(result)
# 我认为跑步最重要的就是给我带来了身体健康
