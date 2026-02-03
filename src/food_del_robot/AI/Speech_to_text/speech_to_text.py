import speech_recognition as sr

r = sr.Recognizer()

while True:
    try:
        # Use the device index explicitly to access the microphone through ALSA
        with sr.Microphone(device_index=1) as source:  # Update device_index based on your system
            print("Listening...")
            
            # Adjust for ambient noise
            r.adjust_for_ambient_noise(source, duration=0.2)
            
            # Listen for audio
            audio = r.listen(source)
            
            # Recognize speech using Google Web Speech API
            text = r.recognize_google(audio)
            text = text.lower()  # Convert to lowercase for uniformity
            
            print("You said:", text)
            
            # Exit the loop if "exit" is mentioned
            if "exit" in text:
                print("Exiting program...")
                break

    except sr.RequestError as e:
        print(f"Could not request results; {e}")
    except sr.UnknownValueError:
        print("Could not understand audio")
    except KeyboardInterrupt:
        print("Program terminated by user")
        break
