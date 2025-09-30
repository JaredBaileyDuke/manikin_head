import sounddevice as sd
from scipy.io.wavfile import write

# Parameters
duration = 10  # seconds
sample_rate = 44100  # Hz
channels = 1  # Mono. Use 2 for stereo
output_filename = 'recording.wav'

print("Recording started...")

# Record audio
audio = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=channels, dtype='int16')
sd.wait()  # Wait until recording is finished

# Save as WAV file
write(output_filename, sample_rate, audio)

print(f"Recording saved as {output_filename}")
