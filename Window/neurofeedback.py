import pandas as pd
import time
import numpy as np
import utils

BUFFER_LENGTH = 5
EPOCH_LENGTH = 1
OVERLAP_LENGTH = 0.8
SHIFT_LENGTH = EPOCH_LENGTH - OVERLAP_LENGTH
INDEX_CHANNEL = [1, 2, 3, 4]

class Band:
    Delta = 0
    Theta = 1
    Alpha = 2
    Beta = 3


if __name__ == "__main__":
    eeg_data = pd.read_csv('CSVs/toby.csv')

    fs = 256
    epoch_buffer = np.zeros((int(fs * EPOCH_LENGTH), len(INDEX_CHANNEL)))
    filter_state = None

    n_win_test = int(np.floor((BUFFER_LENGTH - EPOCH_LENGTH) / SHIFT_LENGTH + 1))
    band_buffer = np.zeros((n_win_test, 4))

    try:
        for _, row in eeg_data.iterrows():
            eeg_samples = row[INDEX_CHANNEL].values
            timestamp = row[0]

            epoch_buffer, filter_state = utils.update_buffer(
                epoch_buffer, eeg_samples, notch=True, filter_state=filter_state)

            if len(epoch_buffer) == fs * EPOCH_LENGTH:
                data_epoch = epoch_buffer
                band_powers = utils.compute_band_powers(data_epoch, fs)
                band_buffer, _ = utils.update_buffer(band_buffer, np.asarray([band_powers]))

                smooth_band_powers = np.mean(band_buffer, axis=0)

                print('Delta:', band_powers[Band.Delta],
                      'Theta:', band_powers[Band.Theta],
                      'Alpha:', band_powers[Band.Alpha],
                      'Beta:', band_powers[Band.Beta])

                # Uncomment the desired neurofeedback protocols
                """
                alpha_metric = smooth_band_powers[utils.Band.Alpha] / smooth_band_powers[utils.Band.Delta]
                print('Alpha Relaxation:', alpha_metric)

                beta_metric = smooth_band_powers[utils.Band.Beta] / smooth_band_powers[utils.Band.Theta]
                print('Beta Concentration:', beta_metric)

                theta_metric = smooth_band_powers[utils.Band.Theta] / smooth_band_powers[utils.Band.Alpha]
                print('Theta Relaxation:', theta_metric)
                """

                print("")

                epoch_buffer = np.zeros((int(fs * EPOCH_LENGTH), len(INDEX_CHANNEL)))

    except KeyboardInterrupt:
        print('Closing!')
