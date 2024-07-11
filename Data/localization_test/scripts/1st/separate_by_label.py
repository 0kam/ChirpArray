import numpy as np
import pandas as pd
import soundfile as sf

dt = "20230621_1050"

for mic in ["micA", "micB"]:
  label_path = "recordings/source/{}.txt".format(dt)
  audio_path = "recordings/source/{}/{}.wav".format(mic, dt)
  
  dt2cond = {
    "20230621_1040": "0deg5m",
    "20230621_1045": "0deg10m",
    "20230621_1050": "0deg20m",
    "20230621_1245": "45deg5m",
    "20230621_1250": "45deg10m"
  }
  
  out = "recordings/separated/{}".format(dt2cond[dt])
  
  audio, rate = sf.read(audio_path)
  
  # ラベルが始まる時間。音源がラベルより進んでいる場合は正の値、遅れている場合は負の値
  # 以下はMicBにのみ適用する（ラベルはMicAでつける）
  
  if (mic == "micA") | (dt == "20230621_1245") | (dt == "20230621_1250"):
    offset = 0
  elif dt == "20230621_1050":
    offset = 50.687 - 51.226 # 20230621_1050
  elif dt == "20230621_1045":
    offset = 8.764 - 18.197 # 20230621_1045
  elif dt == "20230621_1040":
    offset = 14.668-32.953 # 20230621_1040
  
  
  label = pd.read_table(label_path, header=None, names=["start", "end", "label"])
  label["start"] = ((label["start"] + offset) * rate).astype(int)
  label["end"] = ((label["end"] +offset) * rate).astype(int)
  
  for _, s, e, c in label.itertuples():
      a = audio[s:e, :]
      o = f"{out}/{c}/{mic}.wav"
      sf.write(o, a, rate)

