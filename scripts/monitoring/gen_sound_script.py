import numpy as np
import soundfile as sf
import os
import argparse

def generate_waveform(waveform_type, frequency, duration, sample_rate, amplitude):
    t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    if waveform_type == 0:  # Sine
        return amplitude * np.sin(2 * np.pi * frequency * t)
    elif waveform_type == 1:  # Square
        return amplitude * np.sign(np.sin(2 * np.pi * frequency * t))
    elif waveform_type == 2:  # Triangle
        return amplitude * (2 * np.abs(2 * ((t * frequency) % 1) - 1) - 1)
    elif waveform_type == 3:  # Sawtooth
        return amplitude * (2 * ((t * frequency) % 1) - 1)
    else:
        raise ValueError("Invalid waveform type. Use 0: sine, 1: square, 2: triangle, 3: sawtooth.")

def generate_beep_wav(filename, frequency=440, duration=1.0, sample_rate=44100, amplitude=0.5, waveform_type=0, silence=0.0):
    """
    Generate a beep sound and save as a WAV file, with optional silence at the end.
    Args:
        filename (str): Output WAV file name.
        frequency (float): Frequency of the beep in Hz.
        duration (float): Duration of the beep in seconds.
        sample_rate (int): Sampling rate in Hz.
        amplitude (float): Amplitude of the beep (0.0 to 1.0).
        waveform_type (int): 0=sine, 1=square, 2=triangle, 3=sawtooth
        silence (float): Optional silence to append at the end, in seconds.
    """
    waveform = generate_waveform(waveform_type, frequency, duration, sample_rate, amplitude)
    if silence > 0:
        silence_samples = int(sample_rate * silence)
        silence_arr = np.zeros(silence_samples, dtype=waveform.dtype)
        waveform = np.concatenate([waveform, silence_arr])
    sf.write(filename, waveform, sample_rate)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a beep sound WAV file.")
    parser.add_argument('--frequency', type=float, default=440, help='Frequency in Hz (default: 440)')
    parser.add_argument('--duration', type=float, default=1.0, help='Duration in seconds (default: 1.0)')
    parser.add_argument('--amplitude', type=float, default=0.5, help='Amplitude (0.0 to 1.0, default: 0.5)')
    parser.add_argument('--sample_rate', type=int, default=44100, help='Sample rate in Hz (default: 44100)')
    parser.add_argument('--output_name', type=str, default="beep.wav", help='Output file name (WAV format, default: beep.wav)')
    parser.add_argument('--output_dir', type=str, default=os.path.expanduser('~/autoware_mini_ws/src/autoware_mini/nodes/platform/monitoring'), help='Output directory (default: ~/autoware_mini_ws/src/nodes/platform/monitoring)')
    parser.add_argument('--waveform', type=int, default=0, choices=[0,1,2,3], help='Waveform type: 0=sine (default), 1=square, 2=triangle, 3=sawtooth')
    parser.add_argument('--silence', type=float, default=0.0, help='Optional silence to append at the end, in seconds (default: 0.0)')
    args = parser.parse_args()

    # Ensure output directory exists
    os.makedirs(args.output_dir, exist_ok=True)

    # Construct the output path
    output_path = os.path.join(args.output_dir, args.output_name)

    generate_beep_wav(output_path, frequency=args.frequency, duration=args.duration, amplitude=args.amplitude, sample_rate=args.sample_rate, waveform_type=args.waveform, silence=args.silence)