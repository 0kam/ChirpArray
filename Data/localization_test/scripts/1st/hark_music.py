import hark
import numpy as np
import soundfile as sf

winlen = 512
shiftlen = 160

audio, rate = sf.read('./recordings/separated/0deg20m/karasu/micA.wav', dtype = np.float32)

frames = np.lib.stride_tricks.sliding_window_view(audio, winlen, axis = 0)[::shiftlen, :, :]

multifft = hark.node.MultiFFT()
multifft_out = multifft(INPUT=frames)

music = hark.node.LocalizeMUSIC()
loadcm = hark.node.CMLoad()
loadcm_out = loadcm(
    FILENAMER = "./HARK/NoiseCM.zip",
    OPERATION_FLAG = 1
)

music_out = music(
    INPUT=multifft_out.OUTPUT,
    NOISECM=
)