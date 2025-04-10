import pyttsx3 
import time
engine = pyttsx3.init()
engine.setProperty('rate', 200)
# engine.say("前方左转")
# engine.runAndWait()

# time.sleep(1)
engine.say("前方有楼梯，请注意安全")
engine.runAndWait()

