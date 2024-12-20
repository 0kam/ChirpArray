from glob import glob
import torchaudio
import os
from torchaudio.functional import highpass_biquad, lowpass_biquad
from pathlib import Path
import torch
from scipy.signal import resample
import numpy as np
from numpy.lib.stride_tricks import sliding_window_view
import math

# For preparing wav files for the localization test
def preprocess_segments(path, out, sr_to=16000):
    wav, sr = torchaudio.load(path)
    # stereo to mono
    if wav.shape[0] == 2:
        wav = wav.mean(axis=0, keepdim=True)
    # high-pass filter
    out_dir = Path(out).parent
    sp = out_dir.parts[-1]
    if sp != 'other_sounds':
        wav = highpass_biquad(wav, sr, cutoff_freq=300)
    # resample
    wav = wav.numpy()[0]
    wav = resample(wav, math.floor(
        wav.shape[0] / sr * sr_to
    ))
    # normalize
    wav = torch.tensor(wav).unsqueeze(0)
    wav = wav / wav.abs().max() * 0.8
    # save as 16-bit PCM
    wav = (wav * 32767).to(torch.int16)
    out_dir = Path(out).parent
    if out_dir.exists() == False:
        os.makedirs(out_dir)
    torchaudio.save(out, wav, sr_to)

files = glob('Projects/SpresenseOutdoorMicArray/localization_test/wavs/sources/*.wav')
out_files = [f.replace('sources', 'normalized') for f in files]
out_files = [f.replace(Path(f).suffix, '.wav') for f in out_files]

for f, out in zip(files, out_files):
    preprocess_segments(f, out)


files = glob('Projects/SpresenseOutdoorMicArray/localization_test/wavs/sources/*.mp3')
out_files = [f.replace('sources', 'normalized') for f in files]
out_files = [f.replace(Path(f).suffix, '.wav') for f in out_files]

for f, out in zip(files, out_files):
    preprocess_segments(f, out)