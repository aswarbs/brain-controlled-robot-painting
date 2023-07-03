import mne
import numpy as np
from mne.time_frequency import psd_array_welch

# in CSV display, create the subset, instead of passing in start_idx, pass in the subset

def calculate_psd_recursive(data):
    subset_raw = mne.io.RawArray(data.T, info)
    psd, freqs = psd_array_welch(subset_raw.get_data(), sfreq=sfreq, fmin=0.5, fmax=sfreq / 2, n_fft=256)

    # Extract PSD values for each channel and frequency range
    channel_names = subset_raw.ch_names
    freq_band_psd = {}
    for freq_band, (fmin, fmax) in freq_ranges.items():
        freq_mask = (freqs >= fmin) & (freqs <= fmax)
        psd_band = np.mean(psd[:, freq_mask], axis=1)
        freq_band_psd[freq_band] = psd_band

    psd_values[f'{0}-{len(data)}'] = freq_band_psd

# Load the CSV file and remove the first column
data = np.genfromtxt('muse_data.csv', delimiter=',', skip_header=1)[:, 1:]

# Define the sampling frequency (in Hz)
sfreq = 256  # Replace with your actual sampling frequency

# Create MNE RawArray object
info = mne.create_info(ch_names=['TP9', 'AF7', 'AF8', 'TP10'], sfreq=sfreq)
raw = mne.io.RawArray(data.T, info)

# Define the frequency ranges for PSD calculation
freq_ranges = {'alpha': (8, 12), 'beta': (13, 30), 'theta': (4, 7)}

# Calculate PSD recursively for each frequency range and subset of data
psd_values = {}
calculate_psd_recursive(data)

# Print the calculated mean PSD values for each subset of data
for data_range, freq_band_psd in psd_values.items():
    print(f'PSD values for data range: {data_range}')
    for freq_band, psd_band in freq_band_psd.items():
        mean_psd = np.mean(psd_band)
        print(f'Frequency Band: {freq_band}, Mean PSD: {mean_psd}')
