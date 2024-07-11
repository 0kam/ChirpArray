import pyroomacoustics as pra
import numpy as np
import matplotlib.pyplot as plt
import soundfile as sf
from glob import glob
import pandas as pd

# Mic Positions
R = np.array([[0.0, 0.04], [0.04, 0.0], [0.0, -0.04], [-0.04, 0.0]]).T
# Parameters
nfft = 2048
freq_range = [600, 7000]

def get_doa(audio_path, plot=False):
    audio, fs = sf.read(audio_path)
    X = pra.transform.stft.analysis(audio, nfft, nfft // 2)
    X = X.transpose([2, 1, 0])
    doa = pra.doa.NormMUSIC(R, fs, nfft, num_src=1, )
    #doa = pra.doa.FRIDA(R, fs, nfft, num_src=1)
    doa.locate_sources(X, freq_range=freq_range)

    # DOAを取得し、radianからdegreeに変換
    azimuth_recon = (doa.azimuth_recon / np.pi * 180)[-1]
    
    if plot:
        doa.polar_plt_dirac()
        plt.show()
    return azimuth_recon

conditions = glob("*", root_dir="recordings/separated/1st")
sounds = glob("*", root_dir="recordings/separated/1st/45deg10m/")

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
            audio_path = f"recordings/separated/1st/{c}/{s}/{mic}.wav"
            doa = get_doa(audio_path)
            print(f"{mic}: {doa}")
            res["condition"].append(c)
            res["sound"].append(s)
            res["mic"].append(mic)
            res["doa"].append(doa)

pd.DataFrame(res).to_csv("scripts/1st/doa.csv", index=False)
