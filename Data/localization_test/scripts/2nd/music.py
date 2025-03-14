import pyroomacoustics as pra
import numpy as np
import matplotlib.pyplot as plt
import soundfile as sf
from glob import glob
import pandas as pd

# Parameters
nfft = 2048

def get_doa(audio_path, R, sp):
    # define frequency range based on species
    # to be used in the DOA estimation (kHz)
    if sp == "aburazemi":
        freq_range = [4000, 7000]
    elif sp == "higurashi":
        freq_range = [1700, 7000]
    elif sp == "karasu":
        freq_range = [1050, 2600]
    elif sp == "shijukara":
        freq_range = [3600, 7000]
    elif sp == "hikigaeru":
        freq_range = [450, 1000]
    elif sp == "amagaeru":
        freq_range = [1050, 3500]
    else:
        raise ValueError("Invalid species")
    audio, fs = sf.read(audio_path)
    X = pra.transform.stft.analysis(audio, nfft, nfft // 2)
    X = X.transpose([2, 1, 0])
    # NormMUSIC methods for DOA estimation
    doa = pra.doa.NormMUSIC(R, fs, nfft, num_src=2, )
    doa.locate_sources(X, freq_range=freq_range)
    # DoAs
    ## right: 0deg, front: 90deg, left: 180deg, back: 270deg
    azimuth_recon = (doa.azimuth_recon / np.pi * 180)[-1]
    return azimuth_recon

conditions = glob("*", root_dir="recordings/separated/2nd/micL/")
sounds = glob("*", root_dir="recordings/separated/2nd/micL/0deg5m/")

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
        for mic in ["micL", "micR"]:
            if mic == "micL":
                # Mic Positions
                # Below is the correct positions of the microphones
                R = np.array([[0.0, 0.04], [0.04, 0.0], [0.0, -0.04], [-0.04, 0.0]]).T
            else:
                # I missplaced the microphones in the right ChirpArray (micR)
                R = np.array([[0.0, 0.04], [-0.04, 0.0], [0.0, -0.04], [0.04, 0.0]]).T
            audio_path = f"recordings/separated/2nd/{mic}/{c}/{s}"
            # DoA 
            doa = get_doa(audio_path, R, s.replace(".wav", ""))
            print(f"{mic}: {doa}")
            res["condition"].append(c)
            res["sound"].append(s)
            res["mic"].append(mic)
            res["doa"].append(doa)

pd.DataFrame(res).to_csv("scripts/2nd/doa.csv", index=False)
