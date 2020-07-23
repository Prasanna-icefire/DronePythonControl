'''
mytext = 'Welcome to geeksforgeeks!'
  
# Language in which you want to convert 
language = 'en'
  
# Passing the text and language to the engine,  
# here we have marked slow=False. Which tells  
# the module that the converted audio should  
# have a high speed 
myobj = gTTS(text=mytext, lang=language, slow=False) 
  
# Saving the converted audio in a mp3 file named 
# welcome  
myobj.save("welcome.mp3") 
  
# Playing the converted file 
os.system("mpg321 welcome.mp3")
'''
import pyttsx3 
  
# Initialize the converter 
converter = pyttsx3.init() 
  
# Set properties before adding 
# Things to say 
  
# Sets speed percent  
# Can be more than 100 
converter.setProperty('rate', 150) 
# Set volume 0-1 
converter.setProperty('volume', 0.7) 
  
# Queue the entered text  
# There will be a pause between 
# each one like a pause in  
# a sentence 
converter.say("Hello GeeksforGeeks") 

  
# Empties the say() queue 
# Program will not continue 
# until all speech is done talking 
converter.runAndWait() 
