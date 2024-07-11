import pyroomacoustics as pra
import numpy as np
import matplotlib.pyplot as plt
import soundfile as sf
from glob import glob
import pandas as pd

# Mic Positions
R = np.array([[0.0, 0.04], [0.04, 0.0], [0.0, -0.04], [-0.04, 0.0]]).T
# Parameters
nfft = 512
freq_range = [400, 8000]

def get_doa(audio_path):
    audio, fs = sf.read(audio_path)
    X = pra.transform.stft.analysis(audio, nfft, nfft // 2)
    X = X.transpose([2, 1, 0])
    doa = pra.doa.NormMUSIC(R, fs, nfft, num_src=1, )
    #doa = pra.doa.FRIDA(R, fs, nfft, num_src=1)
    azimuths = []
    for i in range(X.shape[2]):
        x = X[:, :, i][:, :, None]
        doa.locate_sources(x, freq_range=freq_range)
    # DOAを取得し、radianからdegreeに変換
        azimuths.append((doa.azimuth_recon / np.pi * 180)[0])
    azimuth_recon = np.median(azimuths)
    return azimuth_recon

conditions = glob("*", root_dir="recordings/separated/")
sounds = glob("*", root_dir="recordings/separated/45deg10m/")

res = {
    "condition": [],
    "sound": [],
    "mic": [],
    "doa": []
}

for c in sorted(conditions):
    print(f"{c}---------------------")
    for s in sounds:
        print(s)
        for mic in ["micA", "micB"]:
            audio_path = f"recordings/separated/{c}/{s}/{mic}.wav"
            doa = get_doa(audio_path)
            print(f"{mic}: {doa}")
            res["condition"].append(c)
            res["sound"].append(s)
            res["mic"].append(mic)
            res["doa"].append(doa)

pd.DataFrame(res).to_csv("doa.csv", index=False)