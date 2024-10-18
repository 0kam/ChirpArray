import numpy as np
import pandas as pd
import soundfile as sf
import os

dt2cond = {
  "20231216_1405": "0deg5m",
  "20231216_1410": "0deg10m",
  "20231216_1415": "0deg20m",
  "20231216_1420": "0deg30m",
  "20231216_1425": "30deg5m",
  "20231216_1430": "30deg10m",
  "20231216_1435": "30deg20m",
  "20231216_1440": "30deg30m",
  "20231216_1445": "45deg5m",
  "20231216_1450": "45deg10m",
  "20231216_1455": "45deg20m",
  "20231216_1500": "45deg30m",
  "20231216_1505": "60deg5m",
  "20231216_1510": "60deg10m",
  "20231216_1515": "60deg20m",
  "20231216_1520": "60deg30m"
}

for dt, cond in dt2cond.items():
  for mic in ["micL", "micR"]:
    label_path = f"recordings/source/2nd/labels/micL/{dt}.txt"
    audio_path = f"recordings/source/2nd/{mic}/{dt}.wav"
    out_dir = f"recordings/separated/2nd/{mic}/{cond}"
    if os.path.exists(out_dir):
      pass
    else:
      os.makedirs(out_dir)
    
    audio, rate = sf.read(audio_path)
    
    # Offset

    if (mic == "micR") & (dt == "20231216_1405"):
      offset = 51.209 - 50.222
    else:
      offset = 0
    
    label = pd.read_table(label_path, header=None, names=["start", "end", "label"])
    label["start"] = ((label["start"] + offset) * rate).astype(int)
    label["end"] = ((label["end"] + offset) * rate).astype(int)
    for _, s, e, c in label.itertuples():
        a = audio[s:e, :]
        o = f"{out_dir}/{c}.wav"
        sf.write(o, a, rate)

