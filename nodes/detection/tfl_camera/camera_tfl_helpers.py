
import numpy as np

def convert_signals_to_rois(signals, img_width, img_height, radius_to_roi_factor):

    # create a list out of each signal and put it into list then convert it to numpy array
    signals = np.array([np.array([signal.plId, signal.linkId, signal.u, signal.v, signal.radius * radius_to_roi_factor]) for signal in signals])
    # calculate additional values: u - radius, u + radius, v - radius, v + radius and put them into the array
    signals = np.concatenate((signals, np.array([[signal[2] - signal[4], signal[2] + signal[4], signal[3] - signal[4], signal[3] + signal[4]] for signal in signals])), axis = 1)
    # group signals by plId and find min(u), max(u), min(v), max(v) for each group, keep also linkId and discard the rest of the data
    signals = np.array([[signal[0], signal[1], np.min(signals[signals[:,0] == signal[0], 5]), np.max(signals[signals[:,0] == signal[0], 6]), np.min(signals[signals[:,0] == signal[0], 7]), np.max(signals[signals[:,0] == signal[0], 8])] for signal in signals])
    # keep only unique signals
    rois = np.unique(signals, axis = 0)

    # check for each roi if they are out of the image, set them to the image edge
    rois[:,2] = np.where(rois[:,2] < 0, 0, rois[:,2])
    rois[:,3] = np.where(rois[:,3] > img_width, img_width, rois[:,3])
    rois[:,4] = np.where(rois[:,4] < 0, 0, rois[:,4])
    rois[:,5] = np.where(rois[:,5] > img_height, img_height, rois[:,5])

    return rois